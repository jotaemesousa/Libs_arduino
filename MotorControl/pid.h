/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita and Bruno Antunes on 16/08/2012
*********************************************************************/

#ifndef PID_h
#define PID_h

#include "Arduino.h"

#define STARTING_PID  				0
#define RUNNING_PID   				1
#define INTEGRAL_ERROR_RESET_TIME	5000

#define AUTOTUNE_MAX_VALUES 60

class pid
{
public:
    pid();

    void setGains(double kp, double ki, double kd);

    double getKp() {return kp_;}
    double getKi() {return ki_;}
    double getKd() {return kd_;}

    void setSampleTime(double sample_time);

    void setMaxOutput(int value) {max_output_ = value;}
    void setMinOutput(int value) {min_output_ = value;}
    void setStopThreshould(double value) {stop_threshould_ = fabs(value);}

    void setNewReference(double ref, bool reset = 1);
    double getReference(void){return reference_;}
    double getLastSensor(void){return last_sensor_;}

    void setFilter(double percent){alpha_ = percent;}

    int run(double input);

    void reset(void);

    void setMaxAccumulatedError(double max){ max_acc_error_ = max;}
    double getMaxAccumulatedError(void){ return max_acc_error_;}

    void setPIDid(char i){ id_ = i;}

    void setDebugOn(void){debug_ = true;}
    void setDebugOff(void){debug_ = false;}

    void setMaxADC(int adc);
    void setMinADC(int adc);

    void initSensor(double last_sensor){last_sensor_ = last_sensor;}

private:

    double kp_;
    double ki_;
    double kd_;

    double sample_time_;

    double last_error_;
    double accumulated_error_, max_acc_error_;

    double state_;

    int max_output_, min_output_;
    double alpha_;

    double last_sensor_;

    double reference_;
    bool stop_flag_, new_reference_;
    double stop_threshould_;
    unsigned long int last_millis_;

    char id_;
    bool debug_;

    int max_adc_, min_adc_;
    bool invert_;
};

#endif

// EOF

