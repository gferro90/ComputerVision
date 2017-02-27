/**
 * @file LineFollower.h
 * @brief Header file for class LineFollower
 * @date 23/feb/2017
 * @author pc
 *
 * @copyright Copyright 2015 F4E | European Joint Undertaking for ITER and
 * the Development of Fusion Energy ('Fusion for Energy').
 * Licensed under the EUPL, Version 1.1 or - as soon they will be approved
 * by the European Commission - subsequent versions of the EUPL (the "Licence")
 * You may not use this work except in compliance with the Licence.
 * You may obtain a copy of the Licence at: http://ec.europa.eu/idabc/eupl
 *
 * @warning Unless required by applicable law or agreed to in writing, 
 * software distributed under the Licence is distributed on an "AS IS"
 * basis, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the Licence permissions and limitations under the Licence.

 * @details This header file contains the declaration of the class LineFollower
 * with all of its public, protected and private members. It may also include
 * definitions for inline methods which need to be visible to the compiler.
 */

#ifndef COMPUTERVISION_PROJECTS_LINEFOLLOWER_LINEFOLLOWER_H_
#define COMPUTERVISION_PROJECTS_LINEFOLLOWER_LINEFOLLOWER_H_

/*---------------------------------------------------------------------------*/
/*                        Standard header includes                           */
/*---------------------------------------------------------------------------*/
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
/*                        Project header includes                            */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*                           Class declaration                               */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                        Inline method definitions                          */
/*---------------------------------------------------------------------------*/



#define CAMERA 0

//definitions of HSV color ranges for signal recognition
#define BLUE1 cvScalar(0, 0, 0, 0) //TODO
#define BLUE2 cvScalar(0, 0, 0, 0) //TODO

#define GREEN1 cvScalar(0, 0, 0, 0) //TODO
#define GREEN2 cvScalar(0, 0, 0, 0) //TODO


#define ORANGE1 cvScalar(11, 133, 179, 0)//TODO
#define ORANGE2 cvScalar(128, 190, 235, 0)//TODO

//definition of the line follower band
#define LINE_BAND_X(rows, cols) 10
#define LINE_BAND_Y(rows, cols) ((2 * rows) / 3)
#define LINE_BAND_WIDTH(rows, cols) (cols - 2*(LINE_BAND_X(rows,cols)))
#define LINE_BAND_HEIGHT(rows, cols) (rows / 12)

//definition of the signal radius ranges
#define SIGNAL_MIN_RADIUS 0 //TODO
#define SIGNAL_MAX_RADIUS 20000 //TODO

//definition of left and right lines ranges
#define MAX_WIDTH_LINE(rows, cols) cols//TODO
#define MIN_WIDTH_LINE(rows, cols) 0//TODO

#define MAX_HEIGHT_LINE(rows, cols) rows//TODO
#define MIN_HEIGHT_LINE(rows, cols) 0//TODO

// definition of the references of left and right boundaries
#define X_REF_LEFT(rows, cols) cols/2 //TODO
#define X_REF_RIGHT(rows, cols) cols/2 //TODO

// conditions to recognie better the line contours
#define MIN_CONTOURS_SIZE 8
#define MAX_CONTOURS_SIZE 0xffffffff
// define the controller's parameters
#define KP 10

// define the drive control remapping parameters
#define DRIVE_CONTROL_MIN -5000.0
#define DRIVE_CONTROL_MAX 5000.0
#define DRIVE_PWM_MIN 7
#define DRIVE_PWM_MAX 17

// define the speed control remapping parameters
#define SPEED_CONTROL_MIN -7200.0
#define SPEED_CONTROL_MAX 7200.0
#define SPEED_PWM_MIN 10
#define SPEED_PWM_MAX 20

//define the standard controls
#define SPEED_ZERO_CONTROL 0//(SPEED_CONTROL_MAX+SPEED_CONTROL_MIN)/2
#define DRIVE_ZERO_CONTROL 0//(DRIVE_CONTROL_MAX+DRIVE_CONTROL_MIN)/2
//zero pwm +1
#define SPEED_STANDARD_CONTROL SPEED_ZERO_CONTROL+(SPEED_CONTROL_MAX-SPEED_CONTROL_MIN)/(SPEED_PWM_MAX-SPEED_PWM_MIN)

//define usb port
#define USB_PORT "/dev/ttyACM0"
// define usb baud rate
#define USB_BAUDRATE 115200

enum stati{FOLLOW_RIGHT=0, FOLLOW_LEFT};

#define ASSIGN_SIGNAL_NAME(name, color, function) {name, color##1, color##2, function}


struct ColorRange {
    const char * signalName;
    CvScalar range1;
    CvScalar range2;
    //get the status and returns speed control
    float (*SignalAction)(int&);
};

#include "SignalActions.h"
static ColorRange signalColorRange[] = { ASSIGN_SIGNAL_NAME("TurnRight",GREEN,TurnRightAction), ASSIGN_SIGNAL_NAME("TurnLeft",ORANGE,TurnLeftAction), 0 };



#endif /* COMPUTERVISION_PROJECTS_LINEFOLLOWER_LINEFOLLOWER_H_ */

