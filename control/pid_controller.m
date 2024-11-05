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
 if isempty(icnt)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;
 
% =================== Your code starts here ===================
%% Parameter Initialization
    persistent u1;
    persistent phiDes;
    persistent thetaDes;
    

    kpPhi = 150; kdPhi = 150;
    kpTheta = 150; kdTheta = 150;
    kpPsi = 20; kdPsi = 1;

    kpX = 10; kdX = 10;
    kpY = 10; kdY = 10;
    kpZ = 8; kdZ = 6;

    phi = qd{qn}.euler(1); phiDot = qd{qn}.omega(1);
    theta = qd{qn}.euler(2); thetaDot = qd{qn}.omega(2);
    psi = qd{qn}.euler(3); psiDot = qd{qn}.omega(3);

    p = cos(theta)*phiDot - cos(phi)*sin(theta)*psiDot;
    q = thetaDot + sin(phi)*psiDot;
    r = sin(theta)*phiDot + cos(phi)*cos(theta)*psiDot;

    if mod(icnt, 5) == 0 || icnt == 1 
        accDes1 = kdX*(qd{qn}.vel_des(1) - qd{qn}.vel(1)) + kpX * (qd{qn}.pos_des(1) - qd{qn}.pos(1)) + qd{qn}.acc_des(1);
        accDes2 = kdY*(qd{qn}.vel_des(2) - qd{qn}.vel(2)) + kpY * (qd{qn}.pos_des(2) - qd{qn}.pos(2)) + qd{qn}.acc_des(2);
        
        phiDes = (1/params.grav)*(accDes1*sin(qd{qn}.yaw_des) - accDes2*cos(qd{qn}.yaw_des));
        thetaDes = (1/params.grav)*(accDes1*cos(qd{qn}.yaw_des) + accDes2*sin(qd{qn}.yaw_des));
        
        u1 = params.mass*params.grav - params.mass*(kdZ*qd{qn}.vel(3) + kpZ*(qd{qn}.pos(3) - qd{qn}.pos_des(3)));
    end

    F = u1;

    M = params.I*[kpPhi*(phiDes - phi) + kdPhi*(-p);
                kpTheta*(thetaDes - theta) + kdTheta*(-q);
                kpPsi*(qd{qn}.yaw_des - psi) + kdPsi*(qd{qn}.yawdot_des - r)];

% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end