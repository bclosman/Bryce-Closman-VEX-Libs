#include "PID.h"

int sgn(float input){
    if (input < 0) return -1;
    return 1;
}

PID::PID(float Kp, float Ki, float Kd, float error, float integral_tolerance, float tolerance, float settle_time, float max, float min){
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.error = error;
    pid.integral_tolerance = integral_tolerance;
    pid.tolerance = tolerance;
    pid.settle_time = settle_time;
    pid.time_settled = 0;
    pid.max = max;
    pid.min = min;
    this->sgn_save = sgn(pid.error);
    data.overshoot = fabs(pid.error);
    data.time = 0;
}

float PID::update(float error){
    pid.previous_error = pid.error;
    pid.error = error;

    if(fabs(pid.error) < pid.integral_tolerance) pid.integral += pid.error;
    if(sgn(pid.error) != sgn(pid.previous_error) || pid.error == 0 || fabs(pid.error) > pid.integral_tolerance) pid.integral = 0;

    float output = pid.Kp*pid.error + pid.Ki*pid.integral + pid.Kd*(pid.error - pid.previous_error);

    if(fabs(pid.error) < pid.tolerance) pid.time_settled += 10;
    else pid.time_settled = 0;

    pid.time += 10;

    /* ----- Tuning Data ----- */
        if(sgn_save*pid.error < data.overshoot) data.overshoot = -sgn_save*pid.error;
        data.time = pid.time;
    /* ----- Tuning Data ----- */

    if(output > pid.max) output = pid.max;
    if(output < pid.min) output = pid.min;
    return output;
}

bool PID::settled(){
    if(pid.time_settled >= pid.settle_time) return true;
    return false;
}

PID::tuning_data PID::get_data(){
    return data;
}