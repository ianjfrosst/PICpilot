/*
 * File:   AttitudeManager.h
 * Author: Chris Hajduk
 *
 * Created on January 26, 2014, 5:43 PM
 */

#ifndef ATTITUDEMANAGER_H
#define	ATTITUDEMANAGER_H

//Header Files
#include "OrientationControl.h"
#include "StateMachine.h"
#include "Network/Datalink.h"
#include "../Common/Common.h"

//Bit Mask Bit Shifts
typedef enum {
    PITCH_CONTROL_TYPE      = 0,
    PITCH_CONTROL_SOURCE    = 1,
    ROLL_CONTROL_TYPE       = 2,
    ROLL_CONTROL_SOURCE     = 3,
    THROTTLE_CONTROL_SOURCE = 4,
    UNUSED_FIVE             = 5, // throttle is 2 bits
    ALTITUDE_CONTROL_SOURCE = 6,
    ALTITUDE_CONTROL        = 7,
    HEADING_CONTROL_SOURCE  = 8,
    HEADING_CONTROL         = 9,
    FLAP_CONTROL_SOURCE     = 10,
} CtrlType;

//Bit Mask Resultant Values
#define RATE_CONTROL 0
#define ANGLE_CONTROL 1

#define RC_SOURCE 0
#define GS_SOURCE 1
#define AP_SOURCE 2 // nothing actually uses AP_SOURCE yet

#define HEADING_GS_SOURCE 0
#define HEADING_AP_SOURCE 1
#define ALTITUDE_GS_SOURCE 0
#define ALTITUDE_AP_SOURCE 1

#define CONTROL_ON 1
#define CONTROL_OFF 0

//Misc
#define COMMAND_HISTORY_SIZE 4

/* FUNCTION PROTOTYPES */
/*****************************************************************************
 * Function: void attitudeInit(void)
 *
 * Preconditions: None.
 *
 * Overview: This function initializes the IMU, SPI, Input Capture, and Output
 * Compare modules or devices.
 *
 * Input:   None.
 *
 * Output:  None.
 *
 *****************************************************************************/
void attitudeInit(void);


char checkDMA();
float getAltitude();
int getHeading();
long double getLongitude();
long double getLatitude();
float getRoll();
float getPitch();
float getYaw();
float getRollRate();
float getPitchRate();
float getYawRate();

int getRollAngleSetpoint();
int getPitchAngleSetpoint();
int getPitchRateSetpoint();
int getRollRateSetpoint();
int getYawRateSetpoint();
int getThrottleSetpoint();
int getAltitudeSetpoint();
int getHeadingSetpoint();

void setPitchAngleSetpoint(int setpoint);
void setRollAngleSetpoint(int setpoint);
void setPitchRateSetpoint(int setpoint);
void setRollRateSetpoint(int setpoint);
void setYawRateSetpoint(int setpoint);
void setThrottleSetpoint(int setpoint);
void setAltitudeSetpoint(int setpoint);
void setHeadingSetpoint(int setpoint);

void inputCapture();

int getPitchAngleInput(char source);
int getRollAngleInput(char source);
int getPitchRateInput(char source);
int getRollRateInput(char source);
int getYawRateInput(char source);
int getThrottleInput(char source);
int getAltitudeInput(char source);
int getHeadingInput(char source);

int getFlapInput(char source);

void imuCommunication();

int coordinatedTurn(float pitchRate, int rollAngle);

uint8_t getControlValue(CtrlType type);

/*****************************************************************************
 * Function: void readDatalink(void);
 *
 * Preconditions: The datalink must have been initialized to use this properly.
 *
 * Overview: This function is responsible for reading commands from the datalink.
 * Each command has an associated function identified by the "cmd" parameter of the
 * data struct. Additional functions may be added to the switch statement (up to
 * 256 possible commands).
 *
 * Input:   None.
 *
 * Output:  None.
 *
 *****************************************************************************/
void readDatalink(void);

/*****************************************************************************
 * Function: int writeDatalink(long frequency);
 *
 * Preconditions: The datalink must have been initialized to use this properly.
 *
 * Overview: This function is responsible for writing to the datalink (UART).
 *
 * Input:   None.
 *
 * Output:  An error code indicating if the data was added to the queue successfully.
 *
 *****************************************************************************/
bool writeDatalink(p_priority packet);

void checkHeartbeat();
void checkGPS();

#endif	/* ATTITUDEMANAGER_H */
