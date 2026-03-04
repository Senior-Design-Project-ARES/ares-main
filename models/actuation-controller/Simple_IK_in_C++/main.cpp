//#include "Simple_IK.h"

/*Header file for Simple_IK.h function
Author: Josh Colgrove
Created: 1/21/26
Revised: 1/23/26
*/

//prevent multiple definitions
#ifndef SIMPLE_IK_H
#define SIMPLE_IK_H

//libraries
#include <numbers>

// structure definitions
struct Dimensions {
    /*
    1 = front right (omni)
    2 = front left (omni)
    3 = front left (normal)
    4 = rear right (normal)
    */
    //wheel radii [m]
    double radius_w1;
    double radius_w2;
    double radius_w3;
    double radius_w4;
    //wheel distance from CG [m]
    double d_wheel1_cg;
    double d_wheel2_cg;
    double d_wheel3_cg;
    double d_wheel4_cg;
};

// function definitions
void Simple_IK(double (&wheelspeed)[4],double v_des, double psi_dot_des_deg, double state, const Dimensions& C,double dt);

#endif


int main(){

    //structure definitions
    Dimensions C;
    C.radius_w1 = 0.1;
    C.radius_w2 = 0.1;
    C.radius_w3 = 0.1;
    C.radius_w4 = 0.1;
    C.d_wheel1_cg = 0.2;
    C.d_wheel2_cg = 0.2;
    C.d_wheel3_cg = 0.2;
    C.d_wheel4_cg = 0.2;

    //variable definitions
    double wheel_speed[4] = {};    //[deg/s]
    double v_des = 10; //[m/s]
    double psi_dot_des_deg = 20; //[deg]
    double state = 0;
    double dt = 0.5; //[s]
    
    //calling functions
    Simple_IK(wheel_speed,v_des, psi_dot_des_deg, state, C, dt);

    return 0;
}

void Simple_IK(double (&wheel_speed)[4], double v_des,double psi_dot_des_deg,double state, const Dimensions& C,double dt){
    //defining constants
    const double pi = 3.141592653589793;

    //rear track width: distance between rear left and rear right wheel
    double rear_track = C.d_wheel3_cg + C.d_wheel4_cg; //[m]
    
    //yaw rate to rad/s
    double psi_dot_rad = psi_dot_des_deg * pi/180;

    // linear wheel speeds [m/s]
    double v1 = v_des;
    double v2 = v_des;

    //rear wheels: full yaw contribution
    double v4 = v_des + psi_dot_rad * (rear_track/2);
    double v3 = v_des - psi_dot_rad * (rear_track/2);

    //convert to angular wheel speeds [deg/s] to find wheel speeds
    wheel_speed[0] = (v1/C.radius_w1) * 180/pi;
    wheel_speed[1] = (v2/C.radius_w2) * 180/pi;
    wheel_speed[2] = (v3/C.radius_w3) * 180/pi;
    wheel_speed[3] = (v4/C.radius_w4) * 180/pi;


}