clc
clear all
close all

robot.m = 2;
robot.d = 0.5;
robot.CoM_er = randn*0.00;
robot.CoM_er
robot.I = robot.m*robot.d^2/12;
robot.l0 = 0.3;
robot.l0_er = randn(1,4)*0.00; %1-4,2-4,1-3,2-3
robot.dl_max = 0.1;
robot.dl_d_max = 0.03;
robot.k = 950;
robot.k_er = randn(1,4)*0; %1-4,2-4,1-3,2-3

oldoptions = optimoptions(@fmincon,'MaxIterations',5000,'MaxFunctionEvaluations',5000,'Display','off');
robot.fminc_options = optimoptions(@fminunc,oldoptions);

t1_int = linspace(0, 5, 5000);

t = [];
x = [];

p3_array = [];
p4_array = [];
q_des_ar = [];
dl_array = [];
moment_ar = [];
fr_array = [];

x_0 = [-0.0,0.4,0.05,0.0,0.1,0.0,0.0]';

for i = 1:20

    q_des = flight_control(x_0,robot);

    [t1,x1,p3_ar1,p4_ar1,dl_ar1,robot] = decor_ode_pogostick_flight(t1_int,x_0,q_des,robot);
    t = [t;t1];
    x = [x;x1];
    p3_array = [p3_array, p3_ar1];
    p4_array = [p4_array, p4_ar1];
    dl_array = [dl_array,dl_ar1];
    fr_array = [fr_array,zeros(3,length(t1))];
    moment_ar = [moment_ar,zeros(1,length(t1))];
    if x1(end,2) < 0
        break
    end
    
    t2_int = linspace(t1(end), t1(end)+5,5000);
    x_0 = x1(end,:)';
    p3 = [x_0(1) + robot.d/2*sin(x_0(3) + q_des)
          x_0(2) - robot.d/2*cos(x_0(3) + q_des)
          0];
      
    
    [t2,x2,p3_ar2,p4_ar2,f_r,fr_ar,dl_ar2,mom_ar2,robot] = decor_ode_pogostick_stance(t2_int,x_0,p3,robot);
    
    t = [t;t2];
    x = [x;x2];
    p3_array = [p3_array, p3_ar2];
    p4_array = [p4_array, p4_ar2];
    dl_array = [dl_array,dl_ar2];
    fr_array = [fr_array,fr_ar];
    moment_ar = [moment_ar,mom_ar2];
    if x2(end,2) < 0
        break
    end
    
    t1_int = linspace(t2(end), t2(end)+5,5000);
    x_0 = x2(end,:)';

end

%visualize_tensegrity(robot,x,p3_array,p4_array,fr_array);

fig = figure;
fig.Name = "position";
subplot(2, 2, 1);
hold on
xlabel("t, s");
ylabel("pos, m");
plot(t,x(:,1))
plot(t,x(:,2))
plot(t,x(:,3))
grid on
legend('x','y','theta')

subplot(2, 2, 3);
hold on
xlabel("t, s");
ylabel("vel, m/s");
plot(t,x(:,5))
plot(t,x(:,6))
plot(t,x(:,7))
grid on
legend('dx','dy','dtheta')

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
plot(t,x(:,4)*100)
grid on
legend('theta','moment','dl')
% 
% % fig = figure;
% % fig.Name = "dl";
% % hold on
% % xlabel("t, s");
% % ylabel("dl, m");
% % plot(t,dl_array )
% % legend('dl')