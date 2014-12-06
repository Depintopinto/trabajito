/**
 * @file    MyRobot.h
 * @brief   Controller for a robot to avoid obstacles.
 *
 * @author  Javier de Pinto Hernandez <100284151@alumnos.uc3m.es>
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

    int num_personas;

    double x;
    double y;
    double z;

    int contador;

    int vuelta;
    double gps[3];

    Compass * _my_compass;
    GPS * _my_gps;
    Camera * _forward_camera;
    double _compass_angle;
    double _compass_angle_green[2];

    double _left_speed, _right_speed;
    int metres;
    int back;
    int entrar;
    int persona;
    int termina;

    int following;

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
    * @brief Function that follow the angle we desired
    * @param the desired angle
    * @return
    */
    void follow_compass(double angle);

    /**
    * @brief Function that follow the angle we desired going back
    * @param the desired angle
    * @return
    */
    void follow_compass_back(double angle);

    /**
    * @brief Function with the logic of the controller
    * that allow the robot to avoid an obstable
    * @param
    * @return
    */
    void control_ida();
    void control_vuelta();

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
        LINEA_RECTA_RIGHT,
        LINEA_RECTA_LEFT,
        LINEA_RECTA,
        BRAKE,
        FAST_TURN_AROUND,
        TRUE_FAST_TURN_AROUND,
        ATRAS_RECTA_RIGHT,
        ATRAS_RECTA_LEFT,
        ATRAS_RECTA
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
    * @return
    */

    double convert_bearing_to_degrees(const double* in_vector);

    /**
    * @brief Find and return the number of geen pixels.
    *
    * The green pixels has been obtain by a comparation
    * of the rgb values of the functions imageGetGreen
    * @param the image given by the camera
    * @return number of green pixels
    */
    int escaner(const char unsigned *image);
    void lineaRecta(double angle);
    void dar_vuelta_completa();
    void giro_escaner();
    void media_gps();
    void rescue_person(double angle);

    void atrasRecta(double angle);

};

