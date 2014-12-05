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
    

    num_personas = 0;
    x = 0;
    y = 0;
    z = 0;
    contador = 0;


    vuelta = 0;
    _compass_angle_green[0]= 1000.0;
    _compass_angle_green[1]= 1000.0;
    metres =0;
    back = 0;
    entrar = 0;
    persona =0;
    termina =0;
    following = 0;
    //Read the value of the encoders
    while (step(_time_step) != -1)
    {

        get_distances();
        sum = 0;

        for (int i = 0; i < NUM_DISTANCE_SENSOR; i++)
        {
            sum = sum + _dist_val[i];
        }
        /*Checks if there is any distance sensor detecting a wall
          and if this wall is close enought (200) to change the mode
          from follow_compass to control */


        if ((gps[2] > 8.2) && (num_personas < 2)){

            if ((_compass_angle_green[0] == 1000.0) || (_compass_angle_green[1] == 1000.0))
            {
                giro_escaner();

                mode();
            }
            else
            {
                gps[2] = 10;
                _forward_camera->disable();
                cout<<"desactivo camara"<< endl;
                if (persona == 0)
                {
                    rescue_person(_compass_angle_green[1]);
                    mode();
                }
                else
                {
                    //hay que meter la logica para volver desde la primera persona
                    // a gps_green e irse a por la siguiente
                    rescue_person(_compass_angle_green[0]);
                    mode();
                }


            }
        }
        else
        {
            if ((gps[2] > 8.2) && (num_personas == 2)){
                cout<<"if 2" << endl;
                if (sum <= 200)
                {
                    //Para que siga la direccion hacia abajo
                    follow_compass(-135);
                    mode();
                }
                else
                {
                    //Modificar control para las paredes a la vuelta
                    control();
                    mode();

                }
                // Set the motor speeds
            }
            else
            {
                media_gps();
                cout<<"if 3" << endl;
                if (sum <= 200)
                {
                    follow_compass(45);
                    mode();
                }
                else
                {
                    control();
                    mode();

                }
                // Set the motor speeds
            }
        }
        cout<<"spee izda "<< _left_speed << endl;
        cout<<"spee dcha "<< _right_speed << endl;
        
        cout<<"personas "<< persona << endl;
        cout<<"num_personas "<< num_personas << endl;

        cout<<"Angulo primera persona "<< _compass_angle_green[1] << endl;
        cout<<"Angulo segunda persona "<< _compass_angle_green[0] << endl;

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
        _mode = LINEA_RECTA_RIGHT;
    }
    else {
        if (_compass_angle > (angle + 1)) {
            // Turn left
            _mode = LINEA_RECTA_LEFT;
        }
        else {
            // Move forward
            _mode = LINEA_RECTA;
        }
    }
}

void MyRobot::follow_compass_back(double angle)
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
        _mode = ATRAS_RECTA_LEFT;
    }
    else {
        if (_compass_angle > (angle + 1)) {
            // Turn left
            _mode = ATRAS_RECTA_RIGHT;
        }
        else {
            // Move forward
            _mode = ATRAS_RECTA;
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
    case LINEA_RECTA_RIGHT:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED /1.25;
        break;
    case LINEA_RECTA_LEFT:
        _left_speed = MAX_SPEED /1.25;
        _right_speed = MAX_SPEED;
        break;
    case LINEA_RECTA:
        _left_speed = MAX_SPEED;
        _right_speed = MAX_SPEED;
        break;
    case BRAKE:
        _left_speed = -10;
        _right_speed =-10;
        break;
    case FAST_TURN_AROUND:
        _left_speed = 20;
        _right_speed = -20;
        break;
    case TRUE_FAST_TURN_AROUND:
        _left_speed = 40;
        _right_speed = -40;
        break;
    case ATRAS_RECTA_RIGHT:
        _left_speed = -MAX_SPEED;
        _right_speed = -MAX_SPEED /1.25;
        break;
    case ATRAS_RECTA_LEFT:
        _left_speed = -MAX_SPEED /1.25;
        _right_speed = -MAX_SPEED;
        break;
    case ATRAS_RECTA:
        _left_speed = -MAX_SPEED;
        _right_speed = -MAX_SPEED;
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

void MyRobot::control()
{
    // Read the compass sensor and convert compass bearing vector to angle, in degrees
    const double *compass_val = _my_compass->getValues();
    _compass_angle = convert_bearing_to_degrees(compass_val);

    //If the robot detect the end of a wall at the right side, and the compass point to rigth position
    //the robot start turning rigth
    if((((_dist_val[12]>400)||(_dist_val[11]>400)) && (_dist_val[0]==0) && (_dist_val[15]==0) && (_dist_val[13]==0) && (_dist_val[14]==0)&&(_dist_val[5]==0 || _dist_val[6]==0 || _dist_val[9]==0 || _dist_val[10] ==0)) && (_compass_angle>-179 || _compass_angle<90))
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
    if(((_dist_val[0]>DISTANCE || _dist_val[15]> DISTANCE) || (_dist_val[1]>3*DISTANCE || _dist_val[14]>3*DISTANCE)) && (_dist_val[7]<200 || _dist_val[8]<200))
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
                    _mode = FORWARD;
                    cout << "forward" << endl;
                }
            }
        }
    }
}

//////////////////////////////////////////////


/**
* @brief Busca y devuelve la cantidad de pixeles verdes.
*
* los pixeles verdes lo hallamos a traves de la comparacion de los valores rgb
* obtenidos a partir de las funciones tipo wb_camera_get
* @param image es la imagen captada por la c?mara frontal
* @return n?mero de pixeles verdes
*/
int MyRobot::escaner(const char unsigned *image){


    int r ,g ,b ;
    int x ,y ;
    int cuenta = 0;

    for (x = 119; x <= 121; x++)
        for (y = 0; y<160; y++){
            g = _forward_camera->imageGetGreen(image, 240, x, y);
            r = _forward_camera->imageGetRed(image, 240, x, y);
            b = _forward_camera->imageGetBlue(image, 240, x, y);
            if (g>r)
                if (g > b)
                    cuenta++;
        }


    return cuenta;
}
/**
* @brief Dirige el movimiento del robot para encontrarse con los supervivientes.
*
* Se realiza un control de velocidad sobre las ruedas para que el robot no se desvie de la direccion
* en la que se encuentran los supervivientes.
* @param lw,rw valores totales de los encoders (left_wheel, right_wheel)
* @param lwh,rwh valores de encoder cuando se hizo la detecci?n de los supervivientes
* @return void
*/
void MyRobot::lineaRecta(double angle){
    /**comparamos la posicion historica con la que se tenia cuando se hizo
   la deteccion para no perder el rumbo.*/
    metres++;

    follow_compass(angle);
}

void MyRobot::atrasRecta(double angle){
    /**comparamos la posicion historica con la que se tenia cuando se hizo
   la deteccion para no perder el rumbo.*/

    if(((_compass_angle >= angle-1.25 && _compass_angle < angle + 1.25) && metres>0) || following ==1){
        metres--;
        if(metres>0){
            following = 1;
            follow_compass(angle);
            cout << "angle atras" << angle << endl;
        }else{
            back=0;
            entrar =0;
            persona ++;
            //para que deje de dar la vuelta sobre si mismo
            following =0;
        }
    }else{
        _mode = TURN_AROUND;
    }
}

/**
* @brief movimiento de rotaci?n del robot.
*
* Frena y gira sobre si mismo, hasta que la condicion de j que debe
* acompa?ar a la funcion, le indica que debe continuar.
*
*/

void MyRobot::dar_vuelta_completa()
{
    _mode = FAST_TURN_AROUND;
    termina ++;
}

void MyRobot::giro_escaner()
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

    if (escaner(image) > 20)
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
void MyRobot::rescue_person(double angle)
{

    // Read the sensors
    const double *compass_val = _my_compass->getValues();

    // Convert compass bearing vector to angle, in degrees
    _compass_angle = convert_bearing_to_degrees(compass_val);

    cout<<"Comapss angle "<< _compass_angle<< endl;
    cout<<"angle "<< angle<< endl;

    if((_compass_angle >= angle-1.25 && _compass_angle < angle + 1.25)|| metres>0){

        if ((entrar ==0)&&(vuelta == 1 ||_dist_val[0] > DISTANCE/2 || _dist_val[1] > DISTANCE/2 ||  _dist_val[14] > DISTANCE/2 || _dist_val[15] > DISTANCE/2)){
            vuelta = 1;

            dar_vuelta_completa();

            if ((_compass_angle >= angle-5.25 && _compass_angle < angle)&&(termina > 20))
            {
                num_personas = num_personas + 1;                
                vuelta = 0;
                back =1;
                //entrar es para que salga de dar la vuelta del anterior if
                entrar = 1;
                //para que termine de dar la vuelta de este if
                termina=0;
            }
        }
        else
        {
            if(back== 0){
                lineaRecta(angle);
            }else{
                atrasRecta(angle-180);
            }
        }
    }
    else{
        _mode = TURN_AROUND;
    }

}

void MyRobot::media_gps()
{
    const double *pos;
    

    pos = _my_gps->getValues();
    if (contador < 50)
    {
        x += pos[0];
        y += pos[1];
        z += pos[2];
        contador++;
    }
    if (contador == 50)
    {
        gps[0] = x / 50;
        gps[1] = y / 50;
        gps[2] = z / 50;

        x = 0;
        y = 0;
        z = 0;
        contador = 0;
    }
}

