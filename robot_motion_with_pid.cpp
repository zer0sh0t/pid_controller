#include <time.h>
#include <iostream>
#include "pid/pid_controller.h"

class Robot 
{
    public:
    double position;

    Robot() { this->position = 0.0; } // robot's initial position

    double move(double current, double dt)
    {
        double torque = 0.5 * current;
        double velocity = 3.0 * torque;
        this->position += velocity * dt;

        srand(time(0));
        int num = (rand() % (10 - 0 + 1)) + 0;
        if (num > 5) { this->position += this->position * 0.0001; } // non-linearity
        return this->position;
    }
};

int main()
{
    Robot robot;
    double curr_position = robot.position;

    double Kp_ = 1.0;
    double Ki_ = 0.7;
    double Kd_ = 0.0;
    double set_point_ = 20.0; // we want to move the robot to this position
    double sample_time_ = 4.0;
    double clamp_vals_[] = {-100, 100};
    PIDController pid_ctrl(Kp_, Ki_, Kd_, set_point_, sample_time_, clamp_vals_);

    double start_time = time(NULL);
    double last_time = start_time;
    
    while (time(NULL) - start_time <= 10.0)
    {
        double curr_time = time(NULL);
        double dt = curr_time - last_time;

        double current = pid_ctrl.update(curr_position);
        curr_position = robot.move(current, 2); // here dt = 2
        last_time = curr_time;
    }
    std::cout << "final position of the robot is " << robot.position << std::endl;

    return 0;
}
