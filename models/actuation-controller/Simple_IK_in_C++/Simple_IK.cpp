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