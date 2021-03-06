/**
 * @file PIDController.cpp
 * @brief Source file for class PIDController
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

 * @details This source file contains the definition of all the methods for
 * the class PIDController (public, protected, and private). Be aware that some 
 * methods, such as those inline could be defined on the header file, instead.
 */

/*---------------------------------------------------------------------------*/
/*                         Standard header includes                          */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                         Project header includes                           */
/*---------------------------------------------------------------------------*/

#include "PIDController.h"

/*---------------------------------------------------------------------------*/
/*                           Static definitions                              */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                           Method definitions                              */
/*---------------------------------------------------------------------------*/

PIDController::PIDController(float KpIn,
                             float KiIn,
                             float KdIn,
                             float error_0) :
        Controller() {
    Kp = KpIn;
    Ki = KiIn;
    Kd = KdIn;
    error_1 = error_0;
    integral = 0.;
}

PIDController::~PIDController() {
    // Auto-generated destructor stub for PIDController
    // TODO Verify if manual additions are needed
}

float PIDController::Execute(float error,
                             float dt) {

    float prop = Kp * error;
    float der = 0.;
    if (dt != 0.) {
        der = Kd * (error - error_1) / dt;
        integral += Ki * error * dt;
    }
    return prop + der + integral;
}
