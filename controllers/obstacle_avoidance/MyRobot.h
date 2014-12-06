/**
 * @file    MyRobot.h
 * @brief   Controller for a robot to find people and coming back to the start.
 *
 * @author  Javier de Pinto Hernandez <100284151@alumnos.uc3m.es>
 * @author  Samuel Hernandez Bermejo <100284298@alumnos.uc3m.es>
 * @date    2014-11
 */


#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED 80
#define NUM_DISTANCE_SENSOR 16
#define DISTANCE 100


class MyRobot : public DifferentialWheels {
private:

    int _time_step;
    ///Variable to count the number of person we have rescued
    int num_person;
    ///Variables to control the gps system
    GPS * _my_gps;
    double x;
    double y;
    double z;
    int counter;
    double gps[3];
    double gps_initial[3];
    ///Variables to control the compass system
    Compass * _my_compass;
    double _compass_angle;
    double _compass_angle_green[2];
    ///Variable to get the forward camera
    Camera * _forward_camera;

    double _left_speed, _right_speed;
    ///Variables to get inside and outside of the if else logic of the program
    bool turn;
    int metres;
    back back;
    bool inside;
    bool person;
    int end;
    bool following;
    ///Variables to control de distance sensors
    double _dist_val[NUM_DISTANCE_SENSOR];
    DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

    /**
    * @brief Function to get the values of the distance sensors
    * @param
    * @return
    */
    void get_distances();

    /**
    * @brief Send the mode to the motors
    * @param
    * @return
    */
    void mode();

    /**
    * @brief Function that follow the desired angle
    * @param the desired angle
    * @return
    */
    void follow_compass(double angle);

    /**
    * @brief Function with the logic of the controller that allow the robot to avoid an obstable
    * @param
    * @return
    */
    void control_up();

    /**
    * @brief Function with the logic of the controller that allow the robot to avoid an obstable coming back
    * @param
    * @return
    */
    void control_down();

    /**
    * @enum Mode
    * @brief Enum with every mode uses by the robot.
    * @param
    * @return
    */
    enum Mode {
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_BACK_LEFT,
        TURN_BACK_RIGHT,
        TURN_LEFT_MORE,
        TURN_RIGHT_MORE,
        TURN_AROUND,
        STOP,
        GO_STRAIGHT_RIGHT,
        GO_STRAIGHT_LEFT,
        GO_STRAIGHT,
        FAST_TURN_AROUND
    };

    Mode _mode;

public:

    /**
    * @brief Empty constructor of the class.
    * @param
    * @return
    */
    MyRobot();

    /**
    * @brief Destructor of the class.
    * @param
    * @return
    */
    ~MyRobot();

    /**
    * @brief Function with the logic of the controller
    * @param
    * @return
    */
    void run();

    /**
    * @brief Converting bearing vector from compass to angle (in degrees).
    * @param bearing vector of compass
    * @return degrees
    */
    double convert_bearing_to_degrees(const double* in_vector);

    /**
    * @brief Find and return the number of green pixels.
    *
    * The green pixels has been obtain by a comparation
    * of the rgb values of the functions imageGetGreen
    * @param the image given by the camera
    * @return number of green pixels
    */
    int scaner(const char unsigned *image);

    /**
    * @brief After the robot finds the position of one person, it goes straight to the person position following the desired angle.
    * @param the desired angle
    * @return
    */
    void go_straight(double angle);

    /**
    * @brief Function for rotational movement of the robot.
    * The robot turns around itself to recognise the person.
    * @param
    * @return
    */
    void turn_around_complete();

    /**
    * @brief Function to recognise the angle where the people are placed, turning around itself.
    *
    * We use function scaner to recognise people.
    * @param the image given by the camera
    * @return number of green pixels
    */
    void scaner_turn_around();

    /**
    * @brief Function to calculate the gps average.
    *
    * We use this function to have a more precise gps position.
    * @param
    * @return
    */
    void gps_average();

    /**
    * @brief Function to see the person from the initial position, to go to the person and recognise it.
    * After all the robot comes back to the initial position and do the same movements for the next person.
    *
    * @param Desired angle of the person´s situation
    * @return
    */
    void rescue_person(double angle);

    /**
    * @brief After the robot recognise one person, it goes straight to the initial position of the recognisement process.
    * After this, in the initial position the robot look for the position of the other person.
    * @param Desired angle of the initial position of the recognisement process.
    * @return
    */
    void go_back_straight(double angle);
};
