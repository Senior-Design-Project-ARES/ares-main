function [wheel_speed] = simple_plant_IK(v_des, psi_dot_des_deg, state, C, dt)
    % Kinematic inverse mapping from desired body velocity and yaw-rate
    % to individual wheel angular velocities [deg/s].
    
    r  = mean([C.radius_w1, C.radius_w2, C.radius_w3, C.radius_w4]); % avg wheel radius [m]
    L  = mean([C.d_wheel1_cg + C.d_wheel2_cg, C.d_wheel3_cg + C.d_wheel4_cg]); % track width [m]
    
    psi_dot_rad = psi_dot_des_deg * pi/180;
    
    % Compute linear velocity of left and right sides
    v_right = v_des + (psi_dot_rad * L/2);
    v_left  = v_des - (psi_dot_rad * L/2);
    
    % Convert to angular wheel speeds [deg/s]
    omega_right = (v_right / r) * 180/pi;
    omega_left  = (v_left  / r) * 180/pi;
    
    % Assign wheel speeds (4x1)
    wheel_speed = [omega_right; omega_right; omega_left; omega_left];
end