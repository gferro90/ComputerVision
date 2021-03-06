/**
 * @file PIDController.h
 * @brief Header file for class PIDController
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

 * @details This header file contains the declaration of the class PIDController
 * with all of its public, protected and private members. It may also include
 * definitions for inline methods which need to be visible to the compiler.
 */

#ifndef COMPUTERVISION_PROJECTS_LINEFOLLOWER_PIDCONTROLLER_H_
#define 		COMPUTERVISION_PROJECTS_LINEFOLLOWER_PIDCONTROLLER_H_

/*---------------------------------------------------------------------------*/
/*                        Standard header includes                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                        Project header includes                            */
/*---------------------------------------------------------------------------*/
#include "Controller.h"
/*---------------------------------------------------------------------------*/
/*                           Class declaration                               */
/*---------------------------------------------------------------------------*/

class PIDController: public Controller {
public:
    PIDController(float KpIn=0.,float KiIn=0.,float KdIn=0., float error_0=0.);

    virtual ~PIDController();

    virtual float Execute(float error, float dt=0.);
private:
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float error_1;
};

/*---------------------------------------------------------------------------*/
/*                        Inline method definitions                          */
/*---------------------------------------------------------------------------*/

#endif /* COMPUTERVISION_PROJECTS_LINEFOLLOWER_PIDCONTROLLER_H_ */

