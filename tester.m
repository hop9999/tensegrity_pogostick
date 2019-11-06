clc
clear all
close all

robot.m = 3;
robot.d = 0.6;
robot.I = robot.m*robot.d^2/12;
robot.l0 = 0.1;
robot.dl_max = 0.5;
robot.dl_d_max = 0.05;
robot.k = 350;

theta_p = 0;
x_p = 0;
y_p = 0.5;
q_des = 0.1;
dl = angle2dl(q_des,robot);


CoM = [x_p;y_p;0];

r = [robot.d/2*cos(theta_p)
     robot.d/2*sin(theta_p)
     0];

p1 = CoM + r;
p2 = CoM - r;

p3 = CoM + [robot.d/2*sin(theta_p + q_des)
               -robot.d/2*cos(theta_p + q_des)
               0];
         
p4 = CoM + [-robot.d/2*sin(theta_p + q_des)
               robot.d/2*cos(theta_p + q_des)
               0];
         
x = [x_p,    y_p,   theta_p,    dl,    0,   0,    0];
[x4, f14, f13, f24, f23] = pogostick_static(p1,p2,p3,dl,robot);

moment = (skew(r)*(-f14) + skew(r)*(-f13) + skew(-r)*(-f24) + skew(-r)*(-f23))/robot.I
x4
p4