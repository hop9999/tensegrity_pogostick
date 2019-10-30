clc
clear all
close all

robot.m = 3;
robot.I = robot.m/12;
robot.d = 0.6;
robot.l0 = 0.2;
robot.dl_max = 0.05;
robot.dl_d_max = 1;
robot.k = 550;

t1_int = linspace(0, 5, 2000);

t = [];
x = [];

p3_array = [];
p4_array = [];
q_des_ar = [];
dl_array = [];
moment_ar = [];

x_0 = [-0.0,1.0,0.0,0,-0.5,0.0,0.0]';
for i = 1:8

    q_des = flight_control(x_0,robot);

    [t1,x1,p3_ar1,p4_ar1,dl_ar1,robot] = decor_ode_pogostick_flight(t1_int,x_0,q_des,robot);
    t = [t;t1];
    x = [x;x1];
    p3_array = [p3_array, p3_ar1];
    p4_array = [p4_array, p4_ar1];
    dl_array = [dl_array,dl_ar1];
    moment_ar = [moment_ar,zeros(1,length(t1))];
    if x1(end,2) < 0
        break
    end
    
    t2_int = linspace(t1(end), t1(end)+5,2000);
    x_0 = x1(end,:)';
    p3 = [x_0(1) + robot.d/2*sin(x_0(3) + q_des)
          x_0(2) - robot.d/2*cos(x_0(3) + q_des)
          0];
      
    
    [t2,x2,p3_ar2,p4_ar2,f_r,dl_ar2,mom_ar2,robot] = decor_ode_pogostick_stance(t2_int,x_0,p3,robot);
    
    t = [t;t2];
    x = [x;x2];
    p3_array = [p3_array, p3_ar2];
    p4_array = [p4_array, p4_ar2];
    dl_array = [dl_array,dl_ar2];
    moment_ar = [moment_ar,mom_ar2];
    if x2(end,2) < 0
        break
    end
    
    t1_int = linspace(t2(end), t2(end)+5,2000);
    x_0 = x2(end,:)';

end

%visualize_tensegrity(robot,x,p3_array,p4_array);

fig = figure;
fig.Name = "pos";
subplot(2, 2, 1);
hold on
xlabel("t, s");
ylabel("pos, m");
plot(t,x(:,1))
plot(t,x(:,2))
plot(t,x(:,3))
plot(t,x(:,4))
grid on
legend('x','y','theta','dl')

subplot(2, 2, 3);
hold on
xlabel("t, s");
ylabel("vel, m/s");
plot(t,x(:,5))
plot(t,x(:,6))
plot(t,x(:,7))
plot(t,x(:,4))
grid on
legend('x_d','y_d','theta_d')

subplot(2, 2, 2);
hold on
xlabel("i, n");
ylabel("force, N");
plot(f_r(1,:) )
plot(f_r(2,:) )
plot(f_r(3,:) )
plot(f_r(4,:) )
grid on
legend('f14', 'f13', 'f24', 'f23')

subplot(2, 2, 4);
hold on
xlabel("t, s");
ylabel("moment, Nm");
plot(t,x(:,3)*100)
plot(t,moment_ar' )
plot(t,x(:,4)*350)
grid on
legend('theta','moment','dl')

% fig = figure;
% fig.Name = "dl";
% hold on
% xlabel("t, s");
% ylabel("dl, m");
% plot(t,dl_array )
% legend('dl')