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

#include <Zigbee.h>

#include <ep/ZigbeeCarbonDioxideSensor.h>
#include "configZigbee.h"
#include <ZigbeeEnvCapteur.h>

#include "Adafruit_SGP30.h"

uint8_t button = D6;

// Optional Time cluster variables
struct tm timeinfo;
struct tm *localTime;
int32_t timezone;

//-----------------------------------------------------------------------------
// Variables globales
SemaphoreHandle_t xSbusI2C;
SemaphoreHandle_t xSzigbee;

//-----------------------------------------------------------------------------
// Capteurs
ZigbeeEnvCapteur zbEnvCapteur = ZigbeeEnvCapteur(TEMP_SENSOR_ENDPOINT_NUMBER, PRESSURE_SENSOR_ENDPOINT_NUMBER, &xSbusI2C, &xSzigbee);

Adafruit_SGP30 sgp;

ZigbeeCarbonDioxideSensor zbCO2Sensor = ZigbeeCarbonDioxideSensor(CO2_SENSOR_ENDPOINT_NUMBER);

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

/********************* Arduino functions **************************/
void setup()
{
    Serial.begin(115200);
    // while (!Serial) {}        // Wait

    // Initialisation du sémaphore pour l'I2C
    xSbusI2C = xSemaphoreCreateBinary();
    xSzigbee = xSemaphoreCreateBinary();

    if((xSbusI2C == NULL) || (xSzigbee == NULL)) 
    {
        Serial.printf("Erreur création sémaphore\n");
    }
    else
    {
        xSemaphoreGive(xSbusI2C);
        xSemaphoreGive(xSzigbee);
    }

    Wire.begin();
    zbEnvCapteur.init();

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

    sgp.setHumidity(zbEnvCapteur.getAbsoluteHumidity());

    // Init button switch
    pinMode(button, INPUT_PULLUP);

    zbCO2Sensor.setManufacturerAndModel("STLP", "ZigbeeCO2Sensor");
    zbCO2Sensor.setMinMaxValue(400, 2500);
    zbCO2Sensor.setTolerance(1);
    zbCO2Sensor.addTimeCluster();
    Zigbee.addEndpoint(&zbCO2Sensor);

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
    timeinfo = zbEnvCapteur.timeinfo();
    timezone = zbEnvCapteur.timezone();

    Serial.println("UTC time:");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

    time_t local = mktime(&timeinfo) + timezone;
    localTime = localtime(&local);

    Serial.println("Local time with timezone:");
    Serial.println(localTime, "%A, %B %d %Y %H:%M:%S");

    // Start Temperature sensor reading task
    xTaskCreate(co2_sensor_value_update, "co2_sensor_update", 2048, NULL, 10, NULL);

    // Set reporting interval for temperature measurement in seconds, must be called after Zigbee.begin()
    // min_interval and max_interval in seconds, delta (temp change in 0,1 °C)
    // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
    // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
    // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of temperature change
    zbCO2Sensor.setReporting(0, 10, 0);
    zbEnvCapteur.setReporting(0, 10, 0);
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
        zbEnvCapteur.report();
        zbCO2Sensor.report();
    }
    delay(100);
}
