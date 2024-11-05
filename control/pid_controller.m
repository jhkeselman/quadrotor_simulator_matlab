 function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

persistent gd;
persistent icnt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;
 
% =================== Your code starts here ===================
%% Parameter Initialization
    kp_roll = 10;
    kd_roll = 10;
    kp_pitch = 10;
    kd_pitch = 10;
    kp_yaw = 10;
    kd_yaw = 10;

    kp_x = 5;
    kd_x = 5;
    kp_y = 5;
    kd_y = 5;
    kp_z = 5;
    kd_z = 5;

    pos_err = qd{qn}.pos_des - qd{qn}.pos;
    vel_err = qd{qn}.vel_des - qd{qn}.vel;

    r_ddot_des = qd{qn}.acc_des + [kp_x*pos_err(1); kp_y*pos_err(2); kp_z*pos_err(3)] + [kd_x*vel_err(1); kd_y*vel_err(1); kd_z*vel_err(3)];

    roll_des = (1/params.grav)*(r_ddot_des(1)*sin(qd{qn}.yaw_des) - r_ddot_des(2)*cos(qd{qn}.yaw_des));
    pitch_des = (1/params.grav)*(r_ddot_des(1)*cos(qd{qn}.yaw_des) + r_ddot_des(2)*sin(qd{qn}.yaw_des));

    F = params.mass*(params.grav + qd{qn}.vel_des(3));

    M = params.I*[kp_roll*(roll_des - qd{qn}.euler(1)) + kd_roll*(-qd{qn}.omega(1));
                kp_pitch*(pitch_des - qd{qn}.euler(2)) + kd_pitch*(-qd{qn}.omega(2));
                kp_yaw*(qd{qn}.yaw_des - qd{qn}.euler(3)) + kd_yaw*(qd{qn}.yawdot_des - qd{qn}.omega(3))];

% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end