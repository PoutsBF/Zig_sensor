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

#include <Zigbee.h>
#include <ep/ZigbeeTempSensor.h>
#include <ep/ZigbeePressureSensor.h>
