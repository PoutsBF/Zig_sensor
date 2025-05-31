#include <Arduino.h>

/******************************************************************************
* Stéphane Lepoutère                                                05/2025   *
*
* Bac à sable sur un capteur zigbee
* Gestion I2C protégée par sémaphore
* Lecture
*   * Température
*   * Hygrométrie
*   * Pression
*   * CO2
*
* TODO : encapsulation dans des 
*
******************************************************************************/

#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

// #include <
#include <Zigbee.h>
#include <ep/ZigbeeTempSensor.h>
#include <ep/ZigbeeCarbonDioxideSensor.h>
#include <ep/ZigbeePressureSensor.h>

#include <BME280I2C.h>
#include <Wire.h>

#include "Adafruit_SGP30.h"

/* Zigbee temperature sensor configuration */
#define TEMP_SENSOR_ENDPOINT_NUMBER 10
#define CO2_SENSOR_ENDPOINT_NUMBER 11
#define PRESSURE_SENSOR_ENDPOINT_NUMBER 12
uint8_t button = D6;

// Optional Time cluster variables
struct tm timeinfo;
struct tm *localTime;
int32_t timezone;

//-----------------------------------------------------------------------------
// Variables globales
SemaphoreHandle_t xSbusI2C;

//-----------------------------------------------------------------------------
// Capteurs
BME280I2C bme;        // Default : forced mode, standby time = 1000 ms
                      // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

Adafruit_SGP30 sgp;

ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);
ZigbeeCarbonDioxideSensor zbCO2Sensor = ZigbeeCarbonDioxideSensor(CO2_SENSOR_ENDPOINT_NUMBER);
ZigbeePressureSensor zbPressure = ZigbeePressureSensor(PRESSURE_SENSOR_ENDPOINT_NUMBER);

/************************ Temp sensor *****************************/
static void temp_sensor_value_update(void *arg)
{
    float temp(NAN), hum(NAN), pres(NAN);
    int16_t iPressure = 0;

    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_hPa);

    for (;;)
    {        
        if (xSbusI2C != NULL)
        {
            if(xSemaphoreTake(xSbusI2C, (TickType_t)0))
            {
                bme.read(pres, temp, hum, tempUnit, presUnit);
                xSemaphoreGive(xSbusI2C);
                // Read temperature sensor value
                iPressure = pres;
                Serial.printf("Updated temperature sensor value to %.2f°C avec %.0f %%HR sous %.f %d hPa\r\n", temp, hum, pres, iPressure);
                // Update temperature value in Temperature sensor EP
                zbTempSensor.setTemperature(temp);
                zbTempSensor.setHumidity(hum);
                zbPressure.setPressure(iPressure);
            }
            else
                Serial.println("I2C pour la température non dispo");
        }        
        delay(10000);
    }
}

/************************ CO2 sensor ******************************/
static void co2_sensor_value_update(void *arg)
{
    for (;;)
    {
        if (xSbusI2C != NULL)
        {
            if (xSemaphoreTake(xSbusI2C, (TickType_t)0))
            {
                if (!sgp.IAQmeasure())
                {
                    Serial.println("Measurement failed");
                }
                xSemaphoreGive(xSbusI2C);
                Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
                Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");
                // Update zigbee
                zbCO2Sensor.setCarbonDioxide(sgp.eCO2);
            }
            else
                Serial.println("I2C pour le CO2 non dispo");
        }
        delay(60000);
    }
}

uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));        // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                       // [mg/m^3]
    return absoluteHumidityScaled;
}

/********************* Arduino functions **************************/
void setup()
{
    Serial.begin(115200);
    // while (!Serial) {}        // Wait

    // Initialisation du sémaphore pour l'I2C
    xSbusI2C = xSemaphoreCreateBinary();

    if(xSbusI2C == NULL)
    {
        Serial.printf("Erreur création sémaphore\n");
    }
    else
        xSemaphoreGive(xSbusI2C);

    Wire.begin();

    while (!bme.begin())
    {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }

    switch (bme.chipModel())
    {
        case BME280::ChipModel_BME280:
            Serial.println("Found BME280 sensor! Success.");
            break;
        case BME280::ChipModel_BMP280:
            Serial.println("Found BMP280 sensor! No Humidity available.");
            break;
        default:
            Serial.println("Found UNKNOWN sensor! Error!");
    }

    Serial.println("SGP30 test");

    if (!sgp.begin())
    {
        Serial.println("Sensor not found :(");
        while (1)
            ;
    }
    Serial.print("Found SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);

    float temperature = 22.1; // [°C]
    float pressure = 22.1; // [°C]
    float humidity = 45.2; // [%RH]
    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);
    bme.read(pressure, temperature, humidity, tempUnit, presUnit);

    sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

    // Init button switch
    pinMode(button, INPUT_PULLUP);

    // Optional: set Zigbee device name and model
    zbTempSensor.setManufacturerAndModel("STLP", "ZigbeeTempSensor");
    // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
    zbTempSensor.setMinMaxValue(-40, 85);
    // Optional: Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
    zbTempSensor.setTolerance(0.5);
    // Optional: Time cluster configuration (default params, as this device will revieve time from coordinator)
    zbTempSensor.addTimeCluster();
    zbTempSensor.addHumiditySensor(0, 100, 1);
    // Add endpoint to Zigbee Core
    Zigbee.addEndpoint(&zbTempSensor);

    zbCO2Sensor.setManufacturerAndModel("STLP", "ZigbeeCO2Sensor");
    zbCO2Sensor.setMinMaxValue(400, 2500);
    zbCO2Sensor.setTolerance(1);
    zbCO2Sensor.addTimeCluster();
    Zigbee.addEndpoint(&zbCO2Sensor);

    zbPressure.setManufacturerAndModel("STLP", "ZigbeePressureSensor");
    zbPressure.setMinMaxValue(300, 1100);
    zbPressure.setTolerance(1);
    zbPressure.addTimeCluster();
    Zigbee.addEndpoint(&zbPressure);

    Serial.println("Starting Zigbee...");
    // When all EPs are registered, start Zigbee in End Device mode
    if (!Zigbee.begin())
    {
        Serial.println("Zigbee failed to start!");
        Serial.println("Rebooting...");
        ESP.restart();
    }
    else
    {
        Serial.println("Zigbee started successfully!");
    }
    Serial.println("Connecting to network");
    while (!Zigbee.connected())
    {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    // Optional: If time cluster is added, time can be read from the coordinator
    timeinfo = zbTempSensor.getTime();
    timezone = zbTempSensor.getTimezone();

    Serial.println("UTC time:");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

    time_t local = mktime(&timeinfo) + timezone;
    localTime = localtime(&local);

    Serial.println("Local time with timezone:");
    Serial.println(localTime, "%A, %B %d %Y %H:%M:%S");

    // Start Temperature sensor reading task
    xTaskCreate(temp_sensor_value_update, "temp_sensor_update", 2048, NULL, 10, NULL);
    xTaskCreate(co2_sensor_value_update, "co2_sensor_update", 2048, NULL, 10, NULL);

    // Set reporting interval for temperature measurement in seconds, must be called after Zigbee.begin()
    // min_interval and max_interval in seconds, delta (temp change in 0,1 °C)
    // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
    // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
    // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of temperature change
    zbTempSensor.setReporting(0, 10, 0);
    zbCO2Sensor.setReporting(0, 10, 0);
    zbPressure.setReporting(0, 60, 0);
}

void loop()
{
    // Checking button for factory reset
    if (digitalRead(button) == LOW)
    {        // Push button pressed
        // Key debounce handling
        delay(100);
        int startTime = millis();
        while (digitalRead(button) == LOW)
        {
            delay(50);
            if ((millis() - startTime) > 3000)
            {
                // If key pressed for more than 3secs, factory reset Zigbee and reboot
                Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
                delay(1000);
                Zigbee.factoryReset();
            }
        }
        zbTempSensor.reportTemperature();
        zbTempSensor.reportHumidity();
        zbCO2Sensor.report();
        zbPressure.report();
    }
    delay(100);
}
