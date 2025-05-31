#pragma once
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

#include <Arduino.h>

#include <Zigbee.h>
#include <ep/ZigbeeTempSensor.h>
#include <ep/ZigbeePressureSensor.h>

#include <BME280I2C.h>
#include <Wire.h>

// #include <configZigbee.h>

class ZigbeeEnvCapteur
{
    public:
        ZigbeeEnvCapteur(uint8_t _endpointTemp, uint8_t _endpointPress, SemaphoreHandle_t * _xSbusI2C, SemaphoreHandle_t * _xSzigbee);

        bool init();
        uint32_t getAbsoluteHumidity();
        struct tm timeinfo();
        int32_t timezone();

        void setReporting(uint16_t min_interval, uint16_t max_interval, uint16_t delta);
        bool report();

        static void task_update(void *arg);

    private:
        uint8_t endpointTemp;
        uint8_t endpointHum;
        ZigbeeCore *zigbee;
        SemaphoreHandle_t *xSbusI2C;
        SemaphoreHandle_t *xSzigbee;
        BME280I2C *bme;        // Default : forced mode, standby time = 1000 ms
        ZigbeeTempSensor *zbTempSensor;
        ZigbeePressureSensor *zbPressure;
};


// More Fuctions
