#pragma once
#include "vex.h"

class PID{
private:

    struct PID_data{
        float Kp, Ki, Kd, error, integral, previous_error, integral_tolerance, tolerance, max, min;

        int time_settled, settle_time, time;
    } pid;

    int sgn_save;

public:

    struct tuning_data{
        float overshoot, time;
    };

private:

    tuning_data data;

public:

    

    /*  @brief creates a PID object
        @param Kp P constant
        @param Ki I constant
        @param Kd D constant
        @param error current error
        @param integral_tolerance tolerance before I starts
        @param tolerance acceptable tolerance
        @param settle_time length of time the error must be within the tolerance to stop, mS
        @param max  max output in volts
        @param min  min output in volts     */
    PID(float Kp, float Ki, float Kd, float error, float integral_tolerance, float tolerance, float settle_time, float max, float min);

    /*  @brief updates the PID
        @param error current error
        @return PID output */
    float update(float error);

    /*  @brief checks if the PID is settled */
    bool settled();

    /* @brief returns data used when tuning in the form of a struct */
    tuning_data get_data();
};