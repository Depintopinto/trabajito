/**
 * @file    MyRobot.cpp
 * @brief   Controller for a robot to avoid obstacles.
 *
 * @author  Javier de Pinto Hernandez <100284151@alumnos.uc3m.es>
 * @date    2014-11
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
    _my_compass->disable();
    _my_gps->disable();
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }

}

//////////////////////////////////////////////
void MyRobot::run()
{
    double sum;
    

    num_person = 0;
    x = 0;
    y = 0;
    z = 0;
    counter = 0;

    turn = false;
    _compass_angle_green[0]= 1000.0;
    _compass_angle_green[1]= 1000.0;
    metres =0;
    back = false;
    inside = false;
    person = false;
    end = 0;
    following = false;

    gps_initial[2] = 1000.0;
    while (step(_time_step) != -1)
    {
        get_distances();
        sum = 0;
        for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
        {
            sum = sum + _dist_val[i];
        }
        if(gps_initial[2] != 1000.0){
            if (((gps[2]- gps_initial[2])>17) && (num_person < 2)){

                if ((_compass_angle_green[0] == 1000.0) || (_compass_angle_green[1] == 1000.0))
                {
                    scaner_turn_around();
                }
                else
                {
                    gps[2] = 10;
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
                if (num_person == 2){
                    cout<<"navegacion vuelta" << endl;
                    /*Checks if there is any distance sensor detecting a wall
                      and if this wall is close enought (200) to change the mode
                      from follow_compass to control */
                    if (sum <= 200)
                    {
                        //Para que siga la direccion hacia abajo
                        follow_compass_down(-135);
                    }
                    else
                    {
                        //Modificar control para las paredes a la turn
                        control_down();

                    }
                    // Set the motor speeds
                }
                else
                {
                    gps_average();
                    /*Checks if there is any distance sensor detecting a wall
                      and if this wall is close enought (200) to change the mode
                      from follow_compass to control */
                    cout<<" navegacion ida" << endl;
                    if (sum <= 200)
                    {
                        cout << "sigo brujula" << endl;
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
            if(_compass_angle >= 35 && _compass_angle < 55){
                _mode = STOP;
                cout << "Paro" <<endl;
            }else{
                _mode = FAST_TURN_AROUND;
                cout << "giro" <<endl;
            }
        }

        cout << "***gps initial 0 " << gps_initial[0] << endl;
        cout << "***gps initial 1 " << gps_initial[1] << endl;
        cout << "***gps initial 2 " << gps_initial[2] << endl;

        cout << "***gps 2 " << gps[2] << endl;

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
    // Read the sensors
    const double *compass_val = _my_compass->getValues();

    // Convert compass bearing vector to angle, in degrees
    _compass_angle = convert_bearing_to_degrees(compass_val);

    // Print sensor values to console
    cout << "Following compass angle (degrees): " << _compass_angle << endl;

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
    // Read the sensors
    const double *compass_val = _my_compass->getValues();

    // Convert compass bearing vector to angle, in degrees
    _compass_angle = convert_bearing_to_degrees(compass_val);

    // Print sensor values to console
    cout << "Following compass angle (degrees): " << _compass_angle << endl;

    // Control the direction to the desired angle
    if (_compass_angle < (angle - 1) || _compass_angle >90)
    {
        // Turn right
        _mode = GO_STRAIGHT_RIGHT;
    }
    else {
        if (_compass_angle > (angle + 1) ) {
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
        cout << "End right wall" << endl;
    }

    //If the robot detect the end of a wall at the left side, and the compass point to rigth position
    //the robot start turning left
    if((((_dist_val[3]>400)||(_dist_val[4]>400))&& (_dist_val[2]==0)&& (_dist_val[0]==0) && (_dist_val[15]==0) && (_dist_val[1]==0) && (_dist_val[14]==0)&&(_dist_val[5]==0 || _dist_val[6]==0 || _dist_val[9]==0 || _dist_val[10] ==0)) && (_compass_angle>0 || _compass_angle<-90))
    {
        _mode = TURN_LEFT_MORE;
        cout << "End left wall" << endl;
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
                cout << "left wall" << endl;
            }
            else
            {
                _mode = TURN_BACK_RIGHT;
                cout << "rigth wall" << endl;
            }

        }
        else
        {
            if(_dist_val[3] > _dist_val[12])
            {
                _mode = TURN_BACK_LEFT;
                cout << "left wall" << endl;
            }
            else
            {
                _mode = TURN_BACK_RIGHT;
                cout << "right wall" << endl;
            }
        }
    }
    else
    {
        //Logic to follow a wall depending if it is too close or too far to the wall
        if(((_dist_val[2]> 4*DISTANCE) || (_dist_val[13]< 3*DISTANCE  && _dist_val[13]!=0)) && (_dist_val[1]==0 || _dist_val[14]==0))
        {
            _mode = TURN_RIGHT;
            cout << "turn right" << endl;
        }
        else
        {
            if((_dist_val[13]> 4*DISTANCE || (_dist_val[2]< 3*DISTANCE && _dist_val[2]!=0)) && (_dist_val[1]==0 || _dist_val[14]==0))
            {
                _mode = TURN_LEFT;
                cout << "turn left" << endl;
            }
            else
            {
                //If the robot detects a wall behind it
                if (_dist_val[7] > 7*DISTANCE || _dist_val[6] > 5*DISTANCE || _dist_val[9] > 5*DISTANCE || _dist_val[8] > 7*DISTANCE)
                {
                    if(_dist_val[0] > 900 || _dist_val[15] > 900){
                        if(_dist_val[0] > _dist_val[15]){
                            _mode = TURN_BACK_LEFT;
                            cout << "atrasito izda" << endl;
                        }else{
                            _mode = TURN_BACK_RIGHT;
                            cout << "atrasito dcha" << endl;
                        }
                    }else{
                        _mode = FORWARD;
                        cout << "forward" << endl;
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
        cout << "End right wall" << endl;
    }

    //If the robot detect the end of a wall at the left side, and the compass point to rigth position
    //the robot start turning left
    if((((_dist_val[3]>400)||(_dist_val[4]>400))&& (_dist_val[2]==0)&& (_dist_val[0]==0) && (_dist_val[15]==0) && (_dist_val[1]==0) && (_dist_val[14]==0)&&(_dist_val[5]==0 || _dist_val[6]==0 || _dist_val[9]==0 || _dist_val[10] ==0)) && (_compass_angle>-179 && _compass_angle<90))
    {
        _mode = TURN_LEFT_MORE;
        cout << "End left wall" << endl;
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
                cout << "left wall" << endl;
            }
            else
            {
                _mode = TURN_BACK_RIGHT;
                cout << "rigth wall" << endl;
            }

        }
        else
        {
            if(_dist_val[3] > _dist_val[12])
            {
                _mode = TURN_BACK_LEFT;
                cout << "left wall" << endl;
            }
            else
            {
                _mode = TURN_BACK_RIGHT;
                cout << "right wall" << endl;
            }
        }
    }
    else
    {
        //Logic to follow a wall depending if it is too close or too far to the wall
        if(((_dist_val[2]> 4*DISTANCE) || (_dist_val[13]< 3*DISTANCE  && _dist_val[13]!=0)) && (_dist_val[1]==0 || _dist_val[14]==0))
        {
            _mode = TURN_RIGHT;
            cout << "turn right" << endl;
        }
        else
        {
            if((_dist_val[13]> 4*DISTANCE || (_dist_val[2]< 3*DISTANCE && _dist_val[2]!=0)) && (_dist_val[1]==0 || _dist_val[14]==0))
            {
                _mode = TURN_LEFT;
                cout << "turn left" << endl;
            }
            else
            {
                //If the robot detects a wall behind it
                if (_dist_val[7] > 7*DISTANCE || _dist_val[6] > 5*DISTANCE || _dist_val[9] > 5*DISTANCE || _dist_val[8] > 7*DISTANCE)
                {
                    if(_dist_val[0] > 900 || _dist_val[15] > 900){
                        if(_dist_val[0] > _dist_val[15]){
                            _mode = TURN_BACK_LEFT;
                            cout << "atrasito izda" << endl;
                        }else{
                            _mode = TURN_BACK_RIGHT;
                            cout << "atrasito dcha" << endl;
                        }
                    }else{
                        _mode = FORWARD;
                        cout << "forward" << endl;
                    }

                }
            }
        }
    }
}

//////////////////////////////////////////////
int MyRobot::scaner(const char unsigned *image){
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
    /**comparamos la posicion historica con la que se tenia cuando se hizo
   la deteccion para no perder el rumbo.*/
    metres++;

    follow_compass(angle);
}

//////////////////////////////////////////////
void MyRobot::go_back_straight(double angle){
    /**comparamos la posicion historica con la que se tenia cuando se hizo
   la deteccion para no perder el rumbo.*/
    cout << "angulo " << angle <<endl;
    if(((_compass_angle >= angle-1.25 && _compass_angle < angle + 1.25) && metres>0) || following ==true){
        metres--;
        if(metres>0){
            following = true;
            follow_compass(angle);
            cout << "angle atras" << angle << endl;
        }else{
            back=false;
            inside = false;
            person =true;
            //para que deje de dar la turn sobre si mismo
            following =false;
        }
    }else{
        cout << "giro porque quiero " <<endl;
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

    // Read the sensors
    const double *compass_val = _my_compass->getValues();

    // Convert compass bearing vector to angle, in degrees
    _compass_angle = convert_bearing_to_degrees(compass_val);

    if (scaner(image) > 20)
    {
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
    // Read the sensors
    const double *compass_val = _my_compass->getValues();

    // Convert compass bearing vector to angle, in degrees
    _compass_angle = convert_bearing_to_degrees(compass_val);

    if((_compass_angle >= angle-1.25 && _compass_angle < angle + 1.25)|| metres >0 ){

        cout << "estoy en el angulo" << endl;
        if ((inside ==false)&&(turn == true ||_dist_val[0] > DISTANCE/2 || _dist_val[1] > DISTANCE/2 ||  _dist_val[14] > DISTANCE/2 || _dist_val[15] > DISTANCE/2)){
            turn = true;
            cout << "Dar vuelta completa" << endl;
            turn_around_complete();

            if ((_compass_angle >= angle-5.25 && _compass_angle < angle)&&(end > 20))
            {
                cout << "vuelta completada" <<endl;
                num_person = num_person + 1;
                turn = false;
                back =true;
                //inside es para que salga de dar la turn del anterior if
                inside = true;
                //para que termine de dar la turn de este if
                end=0;
            }else{
                cout << "elseelse" <<endl;
            }
        }
        else
        {
            if(back== false){
                cout <<"linea recta" << endl;
                go_straight(angle);
            }else{
                cout <<"atras recta" << endl;
                if(angle >=0.0){
                    cout <<"positivo" << endl;
                    go_back_straight(angle-180);
                }else{
                    cout <<"negativo" << endl;
                    go_back_straight(angle+180);
                }

            }
        }
    }
    else if(_compass_angle< angle-20 || _compass_angle> angle +10){
        cout <<"rapido" << endl;
        _mode = FAST_TURN_AROUND;
    } else{
        cout <<"Despacio" << endl;
        _mode = TURN_AROUND;
    }
}

//////////////////////////////////////////////
void MyRobot::gps_average()
{
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

            cout << "gps initial 2 " << gps_initial[2] << endl;
            cout << "gps 2 " << gps[2] << endl;

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
