/**
 * @file IMU.h
 * @author Ian Frosst
 * @date March 16, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 */

#ifndef IMU_H
#define	IMU_H

#include "ProgramStatus.h"
#include "main.h"

#define VN100 0
#define MPU9250 1
#define MPU6050 2

#define USE_IMU VN100

#define IMU_SPI_PORT 2

typedef struct IMUData {
    float roll, pitch, heading;
    float rollRate, pitchRate, yawRate;
} IMUData;

/**
 * Initializes the IMU. This also must initialize the interface for the IMU (SPI, I2C, etc.)
 */
void initIMU(void);

/**
 * Retrieves new orientation data from the IMU.
 */
void updateIMU(void);

void IMU_setOrientation(float*);

void IMU_tare(void);


float getRoll(void);
float getPitch(void);
float getYaw(void);
float getRollRate(void);
float getPitchRate(void);
float getYawRate(void);

#endif
