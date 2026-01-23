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