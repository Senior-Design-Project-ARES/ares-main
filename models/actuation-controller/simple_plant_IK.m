function wheel_speed = simple_plant_IK(v_des, psi_dot_des_deg, state, C, dt)
% IK for 4-motor rover with omni front wheels and normal rear wheels.
% Front wheels: forward only.
% Rear wheels : forward +/- yaw contribution.
%
% wheel order assumed:
%   1 = front right (omni)
%   2 = front left  (omni)
%   3 = rear  left  (normal)
%   4 = rear  right (normal)

    % wheel radii (could be all the same)
    r1 = C.radius_w1;
    r2 = C.radius_w2;
    r3 = C.radius_w3;
    r4 = C.radius_w4;

    % rear track width: distance between rear left and rear right
    rear_track = C.d_wheel3_cg + C.d_wheel4_cg;   % [m]

    % yaw rate to rad/s
    psi_dot_rad = psi_dot_des_deg * pi/180;

    % --- linear wheel speeds [m/s] ---
    % front omnis: no yaw contribution
    v1 = v_des;
    v2 = v_des;

    % rear wheels: full yaw contribution
    v4 = v_des + psi_dot_rad * (rear_track/2);   % rear right
    v3 = v_des - psi_dot_rad * (rear_track/2);   % rear left

    % --- convert to angular wheel speeds [deg/s] ---
    w1 = (v1 / r1) * 180/pi;
    w2 = (v2 / r2) * 180/pi;
    w3 = (v3 / r3) * 180/pi;
    w4 = (v4 / r4) * 180/pi;

    % output as column
    wheel_speed = [w1; w2; w3; w4];
end
