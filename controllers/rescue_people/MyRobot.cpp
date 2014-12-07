/**
 * @file    MyRobot.cpp
 * @brief   Controller for a robot to find people and coming back to the start.
 *
 * @author  Javier de Pinto Hernandez <100284151@alumnos.uc3m.es>
 * @author  Samuel Hernandez Bermejo <100284298@alumnos.uc3m.es>
 * @date    2014-12-07
 */

#include "MyRobot.h"

//////////////////////////////////////////////
MyRobot::MyRobot() : DifferentialWheels()
{
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    // Get and enable the compass device
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);
    // Get and enable the gps device
    _my_gps = getGPS("gps");
    _my_gps->enable(_time_step);
    //Get and enable the distance sensors
    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[1] = getDistanceSensor("ds1");
    _distance_sensor[2] = getDistanceSensor("ds2");
    _distance_sensor[3] = getDistanceSensor("ds3");
    _distance_sensor[4] = getDistanceSensor("ds4");
    _distance_sensor[5] = getDistanceSensor("ds5");
    _distance_sensor[6] = getDistanceSensor("ds6");
    _distance_sensor[7] = getDistanceSensor("ds7");
    _distance_sensor[8] = getDistanceSensor("ds8");
    _distance_sensor[9] = getDistanceSensor("ds9");
    _distance_sensor[10] = getDistanceSensor("ds10");
    _distance_sensor[11] = getDistanceSensor("ds11");
    _distance_sensor[12] = getDistanceSensor("ds12");
    _distance_sensor[13] = getDistanceSensor("ds13");
    _distance_sensor[14] = getDistanceSensor("ds14");
    _distance_sensor[15] = getDistanceSensor("ds15");

    for(int i=0; i<NUM_DISTANCE_SENSOR; i++)
    {
        _distance_sensor[i]->enable(_time_step);
    }

    //Start the robot with the mode FORWARD
    _mode = FORWARD;
}

//////////////////////////////////////////////
MyRobot::~MyRobot()
{
    //Disable compass device and gps device
    _my_compass->disable();
    _my_gps->disable();
    //Disable distance sensors
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////
void MyRobot::run()
{
    double sum;
    
    gps_initial[2] = 1000.0;
    x = 0;
    y = 0;
    z = 0;
    counter = 0;


    _compass_angle_green[0]= 1000.0;
    _compass_angle_green[1]= 1000.0;

    end = 0;
    metres =0;
    num_person = 0;
    turn = false;
    back = false;
    inside = false;
    person = false;
    following = false;

    while (step(_time_step) != -1)
    {
        get_distances();
        sum = 0;
        for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
        {
            sum = sum + _dist_val[i];
        }
        //First we calculate initial gps position, the robot wont start until we know it
        if(gps_initial[2] != 1000.0){
            if (((gps[2]- gps_initial[2])>17) && (num_person < 2)){
                //Start the identification
                if ((_compass_angle_green[0] == 1000.0) || (_compass_angle_green[1] == 1000.0))
                {
                    scaner_turn_around();
                }
                else
                {
                    gps[2] = gps_initial[2] + 18;
                    _forward_camera->disable();
                    if (person == false)
                    {
                        rescue_person(_compass_angle_green[1]);
                    }
                    else
                    {
                        rescue_person(_compass_angle_green[0]);
                    }
                }
            }
            else
            {
                //Start the going down
                if (num_person == 2){
                    /*Checks if there is any distance sensor detecting a wall
                      and if this wall is close enought (200) to change the mode
                      from follow_compass to control */
                    if (sum <= 200)
                    {
                        //To follow down direction of the map
                        follow_compass_down(-135);
                    }
                    else
                    {
                        control_down();
                    }
                }
                else
                {
                    //Start the going up
                    gps_average();
                    if (sum <= 200)
                    {
                        follow_compass(45);
                    }
                    else
                    {
                        control_up();
                    }
                }
            }
        }else{
            gps_average();
            // Read the sensors
            const double *compass_val = _my_compass->getValues();

            // Convert compass bearing vector to angle, in degrees
            _compass_angle = convert_bearing_to_degrees(compass_val);
            //We placed the robot in the correct direction.
            if(_compass_angle >= 35 && _compass_angle < 55){
                _mode = STOP;
            }else{
                _mode = FAST_TURN_AROUND;
            }
        }
        // Set the mode
        mode();
        // Set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////
double MyRobot::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);
    return deg;
}

//////////////////////////////////////////////
void MyRobot::follow_compass(double angle)
{
    const double *compass_val = _my_compass->getValues();
    _compass_angle = convert_bearing_to_degrees(compass_val);

    // Control the direction to the desired angle
    if (_compass_angle < (angle - 1))
    {
        // Turn right
        _mode = GO_STRAIGHT_RIGHT;
    }
    else {
        if (_compass_angle > (angle + 1)) {
            // Turn left
            _mode = GO_STRAIGHT_LEFT;
        }
        else {
            // Move forward
            _mode = GO_STRAIGHT;
        }
    }
}

//////////////////////////////////////////////
void MyRobot::follow_compass_down(double angle)
{
    const double *compass_val = _my_compass->getValues();
    _compass_angle = convert_bearing_to_degrees(compass_val);

    if (_compass_angle < (angle - 1) || _compass_angle >90)
    {
        _mode = GO_STRAIGHT_RIGHT;
    }
    else {
        if (_compass_angle > (angle + 1) ) {
            _mode = GO_STRAIGHT_LEFT;
        }
        else {
            _mode = GO_STRAIGHT;
        }
    }
}

//////////////////////////////////////////////
void MyRobot::mode()
{
    // Send the desired speed mode to the wheels
    switch (_mode){
    case FORWARD:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
        break;
    case TURN_LEFT:
        _left_speed = MAX_SPEED / 1.35;
        _right_speed = MAX_SPEED;
        break;
    case TURN_RIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED / 1.35;
        break;
    case TURN_LEFT_MORE:
        _left_speed = MAX_SPEED / 1.4;
        _right_speed = MAX_SPEED;
        break;
    case TURN_RIGHT_MORE:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED / 1.4;
        break;
    case TURN_BACK_LEFT:
        _left_speed = -MAX_SPEED / 15.0;
        _right_speed = -MAX_SPEED / 5.0;
        break;
    case TURN_BACK_RIGHT:
        _left_speed = -MAX_SPEED / 5.0;
        _right_speed = -MAX_SPEED / 15.0;
        break;
    case TURN_AROUND:
        _left_speed = 5;
        _right_speed = -5;
        break;
    case STOP:
        _left_speed = 0;
        _right_speed = 0;
        break;
    case GO_STRAIGHT_RIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED /1.25;
        break;
    case GO_STRAIGHT_LEFT:
        _left_speed = MAX_SPEED /1.25;
        _right_speed = MAX_SPEED;
        break;
    case GO_STRAIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
        break;    
    case FAST_TURN_AROUND:
        _left_speed = 20;
        _right_speed = -20;
        break;    
    default:
        break;
    }
}

//////////////////////////////////////////////
void MyRobot::get_distances()
{
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++)
    {
        _dist_val[i] = _distance_sensor[i]->getValue();
    }
}

//////////////////////////////////////////////
void MyRobot::control_up()
{
    // Read the compass sensor and convert compass bearing vector to angle, in degrees
    const double *compass_val = _my_compass->getValues();
    _compass_angle = convert_bearing_to_degrees(compass_val);

    //If the robot detect the end of a wall at the right side, and the compass point to rigth position
    //the robot start turning rigth
    if((((_dist_val[12]>400)||(_dist_val[11]>400)) && (_dist_val[0]==0) && (_dist_val[15]==0) && (_dist_val[13]==0) && (_dist_val[14]==0)&&(_dist_val[5]==0 || _dist_val[6]==0 || _dist_val[9]==0 || _dist_val[10] ==0)) && (_compass_angle>-179 && _compass_angle<90))
    {
        _mode = TURN_RIGHT_MORE;
    }

    //If the robot detect the end of a wall at the left side, and the compass point to rigth position
    //the robot start turning left
    if((((_dist_val[3]>400)||(_dist_val[4]>400))&& (_dist_val[2]==0)&& (_dist_val[0]==0) && (_dist_val[15]==0) && (_dist_val[1]==0) && (_dist_val[14]==0)&&(_dist_val[5]==0 || _dist_val[6]==0 || _dist_val[9]==0 || _dist_val[10] ==0)) && (_compass_angle>0 || _compass_angle<-90))
    {
        _mode = TURN_LEFT_MORE;
    }

    //If the robot detect a wall in front
    if(((_dist_val[0]>DISTANCE || _dist_val[15]> DISTANCE) || (_dist_val[1]>3*DISTANCE || _dist_val[14]>3*DISTANCE)) && (_dist_val[7]<300 || _dist_val[8]<300))
    {
        //This if-else choose which side of the robot is near to the wall
        //and turn to that side
        if(_dist_val[3] == 0 && _dist_val[12] == 0)
        {
            if((_dist_val[2] > _dist_val[13]) || (_dist_val[1] > _dist_val[14]) || (_dist_val[0] > _dist_val[15]))
            {
                _mode = TURN_BACK_LEFT;
            }
            else
            {
                _mode = TURN_BACK_RIGHT;
            }

        }
        else
        {
            if(_dist_val[3] > _dist_val[12])
            {
                _mode = TURN_BACK_LEFT;
            }
            else
            {
                _mode = TURN_BACK_RIGHT;
            }
        }
    }
    else
    {
        //Logic to follow a wall depending if it is too close or too far to the wall
        if(((_dist_val[2]> 4*DISTANCE) || (_dist_val[13]< 3*DISTANCE  && _dist_val[13]!=0)) && (_dist_val[1]==0 || _dist_val[14]==0))
        {
            _mode = TURN_RIGHT;
        }
        else
        {
            if((_dist_val[13]> 4*DISTANCE || (_dist_val[2]< 3*DISTANCE && _dist_val[2]!=0)) && (_dist_val[1]==0 || _dist_val[14]==0))
            {
                _mode = TURN_LEFT;
            }
            else
            {
                //If the robot detects a wall behind it
                if (_dist_val[7] > 9*DISTANCE || _dist_val[6] > 7*DISTANCE || _dist_val[9] > 7*DISTANCE || _dist_val[8] > 9*DISTANCE)
                {
                    if(_dist_val[0] > 900 || _dist_val[15] > 900){
                        if(_dist_val[0] > _dist_val[15]){
                            _mode = TURN_BACK_LEFT;
                        }else{
                            _mode = TURN_BACK_RIGHT;
                        }
                    }else{
                        _mode = FORWARD;
                    }
                }
            }
        }
    }
}

//////////////////////////////////////////////
void MyRobot::control_down()
{
    // Read the compass sensor and convert compass bearing vector to angle, in degrees
    const double *compass_val = _my_compass->getValues();
    _compass_angle = convert_bearing_to_degrees(compass_val);

    //If the robot detect the end of a wall at the right side, and the compass point to rigth position
    //the robot start turning rigth
    if((((_dist_val[12]>400)||(_dist_val[11]>400)) && (_dist_val[0]==0) && (_dist_val[15]==0) && (_dist_val[13]==0) && (_dist_val[14]==0)&&(_dist_val[5]==0 || _dist_val[6]==0 || _dist_val[9]==0 || _dist_val[10] ==0)) && (_compass_angle>0 || _compass_angle<-90))
    {
        _mode = TURN_RIGHT_MORE;
    }

    //If the robot detect the end of a wall at the left side, and the compass point to rigth position
    //the robot start turning left
    if((((_dist_val[3]>400)||(_dist_val[4]>400))&& (_dist_val[2]==0)&& (_dist_val[0]==0) && (_dist_val[15]==0) && (_dist_val[1]==0) && (_dist_val[14]==0)&&(_dist_val[5]==0 || _dist_val[6]==0 || _dist_val[9]==0 || _dist_val[10] ==0)) && (_compass_angle>-179 && _compass_angle<90))
    {
        _mode = TURN_LEFT_MORE;
    }

    //If the robot detect a wall in front
    if(((_dist_val[0]>DISTANCE || _dist_val[15]> DISTANCE) || (_dist_val[1]>3*DISTANCE || _dist_val[14]>3*DISTANCE)) && (_dist_val[7]<300 || _dist_val[8]<300))
    {
        //This if-else choose which side of the robot is near to the wall
        //and turn to that side
        if(_dist_val[3] == 0 && _dist_val[12] == 0)
        {
            if((_dist_val[2] > _dist_val[13]) || (_dist_val[1] > _dist_val[14]) || (_dist_val[0] > _dist_val[15]))
            {
                _mode = TURN_BACK_LEFT;
            }
            else
            {
                _mode = TURN_BACK_RIGHT;
            }
        }
        else
        {
            if(_dist_val[3] > _dist_val[12])
            {
                _mode = TURN_BACK_LEFT;
            }
            else
            {
                _mode = TURN_BACK_RIGHT;
            }
        }
    }
    else
    {
        //Logic to follow a wall depending if it is too close or too far to the wall
        if(((_dist_val[2]> 4*DISTANCE) || (_dist_val[13]< 3*DISTANCE  && _dist_val[13]!=0)) && (_dist_val[1]==0 || _dist_val[14]==0))
        {
            _mode = TURN_RIGHT;
        }
        else
        {
            if((_dist_val[13]> 4*DISTANCE || (_dist_val[2]< 3*DISTANCE && _dist_val[2]!=0)) && (_dist_val[1]==0 || _dist_val[14]==0))
            {
                _mode = TURN_LEFT;
            }
            else
            {
                //If the robot detects a wall behind it
                if (_dist_val[7] > 9*DISTANCE || _dist_val[6] > 7*DISTANCE || _dist_val[9] > 7*DISTANCE || _dist_val[8] > 9*DISTANCE)
                {
                    if(_dist_val[0] > 900 || _dist_val[15] > 900){
                        if(_dist_val[0] > _dist_val[15]){
                            _mode = TURN_BACK_LEFT;
                        }else{
                            _mode = TURN_BACK_RIGHT;
                        }
                    }else{
                        _mode = FORWARD;
                    }
                }
            }
        }
    }
}

//////////////////////////////////////////////
int MyRobot::scaner(const char unsigned *image){
    /*We use the forward camera to have Red,Green and Blue and
     we compare the three colours if green is bigger we add one to count*/
    int r ,g ,b ;
    int x ,y ;
    int count = 0;

    for (x = 119; x <= 121; x++)
        for (y = 0; y<160; y++){
            g = _forward_camera->imageGetGreen(image, 240, x, y);
            r = _forward_camera->imageGetRed(image, 240, x, y);
            b = _forward_camera->imageGetBlue(image, 240, x, y);
            if (g>r)
                if (g > b)
                    count++;
        }
    return count;
}

//////////////////////////////////////////////
void MyRobot::go_straight(double angle){
    /*Metres is used to knwo the distance from scaner position to the rescued person
    we are going to use this valuable in go_back_straight too.*/
    metres++;

    follow_compass(angle);
}

//////////////////////////////////////////////
void MyRobot::go_back_straight(double angle){
    /*If the robot angle is between the range and our distance is bigger than 0
    the robot will move to the initial scaner position, When the compass angle isnt in the range,
    the robot will move because metres is bigger than 0 and following its true.*/
    if(((_compass_angle >= angle-1.25 && _compass_angle < angle + 1.25) && metres>0) || following ==true){
        metres--;
        if(metres>0){
            following = true;
            follow_compass(angle);
        }else{
            //The robot doesnt need to move back.
            back=false;
            inside = false;
            //When the robot is in scaner position person is true
            person =true;
            //To not turn around itself(robot)
            following =false;
        }
    }else{
        _mode = TURN_AROUND;
    }
}

//////////////////////////////////////////////
void MyRobot::turn_around_complete()
{
    _mode = FAST_TURN_AROUND;
    end ++;
}

//////////////////////////////////////////////
void MyRobot::scaner_turn_around()
{
    // Get and enable the camera device
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    // Get current image from forward camera
    const unsigned char *image = _forward_camera->getImage();

    const double *compass_val = _my_compass->getValues();
    _compass_angle = convert_bearing_to_degrees(compass_val);
    //If the robot detect something green.
    if (scaner(image) > 20)
    {
        /*Finded angle person, if it is the first person we will safe this angle in _compass_angle_green[1]
        if its the second it will be saved in _compass_angle_green[0]*/
        _compass_angle_green[0] = _compass_angle;
    }
    else
    {
        _mode = FAST_TURN_AROUND;
        if (_compass_angle_green[0] != 1000.0)
        {
            if (_compass_angle_green[1] == 1000.0)
            {
                _compass_angle_green[1] = _compass_angle_green[0]-2;
                _compass_angle_green[0] = 1000.0;
            }
        }
    }
    if(_compass_angle_green[0]!=1000.0 && _compass_angle_green[1]!=1000.0){

        _mode = STOP;
    }
}

//////////////////////////////////////////////
void MyRobot::rescue_person(double angle)
{
    const double *compass_val = _my_compass->getValues();
    _compass_angle = convert_bearing_to_degrees(compass_val);

    if((_compass_angle >= angle-1.25 && _compass_angle < angle + 1.25)|| metres >0 ){
        if ((inside ==false)&&(turn == true ||_dist_val[0] > DISTANCE/2 || _dist_val[1] > DISTANCE/2 ||  _dist_val[14] > DISTANCE/2 || _dist_val[15] > DISTANCE/2)){
            turn = true;
            turn_around_complete();
            if ((_compass_angle >= angle-5.25 && _compass_angle < angle)&&(end > 20))
            {
                num_person = num_person + 1;
                turn = false;
                back =true;
                //To exit of this if.
                inside = true;
                //Until end is 0 the robot is turned around itself.
                end=0;
            }
        }
        else
        {
            if(back== false){
                go_straight(angle);
            }else{
                /*if angle is negative and you try to subtract 180 the result of this is an angle that doesnt exist.*/
                if(angle >=0.0){
                    go_back_straight(angle-180);
                }else{
                    go_back_straight(angle+180);
                }
            }
        }
    }
    else if(_compass_angle< angle-20 || _compass_angle> angle +10){
        _mode = FAST_TURN_AROUND;
    } else{
        _mode = TURN_AROUND;
    }
}

//////////////////////////////////////////////
void MyRobot::gps_average()
{
    /*we calculate the gps position with an average of 50 gps position because if we use only one position gps it cant be right
    Also we calculate initial gps position to know when the robot is up in the map.*/
    const double *pos;
    pos = _my_gps->getValues();

    if(gps_initial[2] == 1000.0){
        if (counter < 50)
        {
            x += pos[0];
            y += pos[1];
            z += pos[2];
            counter++;
        }
        if (counter == 50)
        {
            gps_initial[0] = x / 50;
            gps_initial[1] = y / 50;
            gps_initial[2] = z / 50;
            gps[0] = x / 50;
            gps[1] = y / 50;
            gps[2] = z / 50;

            x = 0;
            y = 0;
            z = 0;
            counter = 0;
        }
    }else{
        if (counter < 50)
        {
            x += pos[0];
            y += pos[1];
            z += pos[2];
            counter++;
        }
        if (counter == 50)
        {
            gps[0] = x / 50;
            gps[1] = y / 50;
            gps[2] = z / 50;
            x = 0;
            y = 0;
            z = 0;
            counter = 0;
        }
    }
}
