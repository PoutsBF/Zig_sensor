/******************************************************************************
 * Stéphane Lepoutère                                                05/2025  *
 *
 * Bac à sable sur un capteur zigbee
 *  Capteur T° / %HR / hPa
 *
 ******************************************************************************/

// #include <configZigbee.h>
#include "ZigbeeEnvCapteur.h"

#define TEMP_SENSOR_ENDPOINT_NUMBER (uint8_t)10
#define PRESSURE_SENSOR_ENDPOINT_NUMBER (uint8_t)12

ZigbeeEnvCapteur::ZigbeeEnvCapteur(uint8_t _endpointTemp, uint8_t _endpointPress, SemaphoreHandle_t *_xSbusI2C, SemaphoreHandle_t *_xSzigbee)
{
    endpointTemp = _endpointTemp;
    endpointHum = _endpointPress;
    xSbusI2C = _xSbusI2C;
    xSzigbee = _xSzigbee;
    bme = new BME280I2C();        // Default : forced mode, standby time = 1000 ms
                                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

    zbTempSensor = new ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);
    zbPressure = new ZigbeePressureSensor(PRESSURE_SENSOR_ENDPOINT_NUMBER);
}

bool ZigbeeEnvCapteur::init()
{
    while (!bme->begin())
    {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }

    switch (bme->chipModel())
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

    // Optional: set Zigbee device name and model
    zbTempSensor->setManufacturerAndModel("STLP", "ZigbeeTempSensor");
    // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
    zbTempSensor->setMinMaxValue(-40, 85);
    // Optional: Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
    zbTempSensor->setTolerance(0.5);
    // Optional: Time cluster configuration (default params, as this device will revieve time from coordinator)
    zbTempSensor->addTimeCluster();
    zbTempSensor->addHumiditySensor(0, 100, 1);
    // Add endpoint to Zigbee Core
    Zigbee.addEndpoint(zbTempSensor);

    zbPressure->setManufacturerAndModel("STLP", "ZigbeePressureSensor");
    zbPressure->setMinMaxValue(300, 1100);
    zbPressure->setTolerance(1);
    zbPressure->addTimeCluster();
    Zigbee.addEndpoint(zbPressure);

    xTaskCreate(ZigbeeEnvCapteur::task_update, "Env_sensor_update", 2048, (void *)this, 10, NULL);

    return false;
}

uint32_t ZigbeeEnvCapteur::getAbsoluteHumidity()
{
    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);

    float temperature = 22.1;        // [°C]
    float pressure = 22.1;           // [°C]
    float humidity = 45.2;           // [%RH]
    bme->read(pressure, temperature, humidity, tempUnit, presUnit);

    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature));        // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                       // [mg/m^3]
    return absoluteHumidityScaled;
}

tm ZigbeeEnvCapteur::timeinfo()
{
    return zbTempSensor->getTime();
}

int32_t ZigbeeEnvCapteur::timezone()
{
    return zbTempSensor->getTimezone();
}

void ZigbeeEnvCapteur::setReporting(uint16_t min_interval, uint16_t max_interval, uint16_t delta)
{
    // Set reporting interval for temperature measurement in seconds, must be called after Zigbee.begin()
    // min_interval and max_interval in seconds, delta (temp change in 0,1 °C)
    // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
    // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
    // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of temperature change
    zbTempSensor->setReporting(min_interval, max_interval, delta);
    zbPressure->setReporting(min_interval, max_interval, delta);
}

bool ZigbeeEnvCapteur::report()
{
    bool resultat = zbTempSensor->reportTemperature() && zbTempSensor->reportHumidity() && zbPressure->report();
    return resultat;
}

void ZigbeeEnvCapteur::task_update(void *arg)
{
    static ZigbeeEnvCapteur *moi;
    moi = (ZigbeeEnvCapteur *)arg;
    float temp(NAN), hum(NAN), pres(NAN);
    int16_t iPressure = 0;

    for (;;)
    {
        if (*moi->xSbusI2C != NULL)
        {
            if (xSemaphoreTake(*moi->xSbusI2C, (TickType_t)0))
            {
                BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
                BME280::PresUnit presUnit(BME280::PresUnit_Pa);

                moi->bme->read(pres, temp, hum, tempUnit, presUnit);
                xSemaphoreGive(*moi->xSbusI2C);
                // Read temperature sensor value
                iPressure = pres;
                Serial.printf("Updated temperature sensor value to %.2f°C avec %.0f %%HR sous %.f %d hPa\r\n", temp, hum, pres, iPressure);

                if(*moi->xSzigbee != NULL)
                {
                    if(xSemaphoreTake(*moi->xSzigbee, (TickType_t)0))
                    {
                        // Update temperature value in Temperature sensor EP
                        Serial.println("Mise à jour Zigbee");
                        moi->zbTempSensor->setTemperature(temp);
                        Serial.println("MAJ température");
                        moi->zbTempSensor->setHumidity(hum);
                        Serial.println("MAJ humidité");
                        moi->zbPressure->setPressure(iPressure);
                        Serial.println("MAJ Pression");
                        xSemaphoreGive(*moi->xSzigbee);
                    }
                    else
                        Serial.println("zigbee pour la température non dispo");
                }
            }
            else
                Serial.println("I2C pour la température non dispo");
        }
        Serial.println("pause température");
        delay(10000);
    }
}
