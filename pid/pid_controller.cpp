#include <time.h>
#include "pid_controller.h"

PIDController::PIDController(double Kp_, double Ki_, double Kd_, double set_point_, double sample_time_, double *clamp_vals_):
                            Kp(Kp_), Ki(Ki_), Kd(Kd_), set_point(set_point_), sample_time(sample_time_), clamp_vals(clamp_vals_)
{
    this->p_term = 0.0;
    this->i_term = 0.0;
    this->d_term = 0.0;
    this->last_error = 0.0;
    this->curr_time = time(NULL);
    this->last_time = curr_time;
    this->last_output = 0.0;
}    

double PIDController::update(double feedback_val)
{
    // output = Kp * error(t) + Ki * integral_0_to_t(error(t) * dt) + Kd * d(error(t)) / dt)
    double error = this->set_point - feedback_val;
    double d_err = error - this->last_error;
    this->curr_time = time(NULL);
    double dt = this->curr_time - this->last_time;

    if (dt >= this->sample_time)
    {
        this->p_term = this->Kp * error;
        this->i_term += error * dt;
        if (dt != 0) { this->d_term = d_err / dt; }

        if (this->i_term < this->clamp_vals[0]) { this->i_term = this->clamp_vals[0]; }
        else if (this->i_term > this->clamp_vals[1]) { this->i_term = this->clamp_vals[1]; }

        this->last_time = this->curr_time;
        this->last_error = error;
        
        double output = this->p_term + (this->Ki * this->i_term) + (this->Kd * this->d_term);
        this->last_output = output;
        return output;
    }
    else
    {
        return this->last_output;
    }
}
