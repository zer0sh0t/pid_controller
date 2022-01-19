#include "time.h"
#include <iostream>
#include "pid/pid_controller.h"

class Robot 
{
    public:
    double position;

    Robot() { this->position = 0.0; } // robot's initial position

    double move(double velocity)
    {
        this->position += velocity * 2.0;
        return this->position;
    }
};

int main()
{
    Robot robot;
    double curr_position = robot.position;

    double Kp_ = 5.0;
    double Ki_ = 2.0;
    double Kd_ = 3.0;
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

        double velocity = pid_ctrl.update(curr_position);
        curr_position = robot.move(velocity);
        last_time = curr_time;
    }
    std::cout << "final position of the robot is " << robot.position << std::endl;

    return 0;
}
