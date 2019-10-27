clc
clear all
close all

robot.m = 3;
robot.I = robot.m/12;
robot.d = 0.6;
robot.l0 = 0.25;
robot.dl = 0;
robot.dl_max = 0.05;
robot.k = 950;

t1_int = linspace(0, 5, 500);

t = [];
x = [];
p3_array = [];
p4_array = [];
q_des_ar = [];
dl_array = [];
x_0 = [-0.0,1.0,0.0,0.1,0.0,0.0]';
for i = 1:3
    theta = x_0(3);
    x_d = x_0(4);
    x_d_des = 0.0;
    ts = 0.12;
    kx = 0.005;
    xf = -x_d*ts/2 + kx*(x_d_des - x_d);
    q_des = -theta - asin(xf/(robot.d/2.3));
	q_des_ar = [q_des_ar,q_des];
    robot.dl
    [t1,x1,p3_ar1,p4_ar1,dl_ar1,robot] = decor_ode_pogostick_flight(t1_int,x_0,q_des,robot);

    t = [t;t1];
    x = [x;x1];
    p3_array = [p3_array, p3_ar1];
    p4_array = [p4_array, p4_ar1];
    dl_array = [dl_array,dl_ar1];
    t2_int = linspace(t1(end), t1(end)+5,500);
    x_0 = x1(end,:)';
    p3 = [x_0(1) + robot.d/2*sin(x_0(3) + q_des)
          x_0(2) - robot.d/2*cos(x_0(3) + q_des)
          0];
      
    if x1(end,2) < 0
        break
    end
    [t2,x2,p3_ar2,p4_ar2,f_r,dl_ar2,robot] = decor_ode_pogostick_stance(t2_int,x_0,p3,robot);
    
    t = [t;t2];
    x = [x;x2];
    p3_array = [p3_array, p3_ar2];
    p4_array = [p4_array, p4_ar2];
    dl_array = [dl_array,dl_ar2];
    t1_int = linspace(t2(end), t2(end)+5,500);
    x_0 = x2(end,:)';
    
    if x2(end,2) < 0
        break
    end

end

visualize_tensegrity(robot,x,p3_array,p4_array);

% fig = figure;
% fig.Name = "pos";
% hold on
% xlabel("t, s");
% ylabel("pos, m");
% plot(t,x(:,1))
% plot(t,x(:,2))
% plot(t,x(:,3))
% legend('x','y','theta')

fig = figure;
fig.Name = "vel";
hold on
xlabel("t, s");
ylabel("vel, m/s");
plot(t,x(:,4))
plot(t,x(:,5))
plot(t,x(:,6))
plot(t,dl_array*5 )
grid on
legend('x_d','y_d','theta_d')

fig = figure;
fig.Name = "forces";
hold on
xlabel("i, n");
ylabel("force, N");
plot(f_r(1,:) )
plot(f_r(2,:) )
plot(f_r(3,:) )
plot(f_r(4,:) )
legend('f14', 'f13', 'f24', 'f23')

% fig = figure;
% fig.Name = "dl";
% hold on
% xlabel("t, s");
% ylabel("dl, m");
% plot(t,dl_array )
% legend('dl')