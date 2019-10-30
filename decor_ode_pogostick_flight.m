function [t,x,p3_ar,p4_ar,dl_ar,robot] = decor_ode_pogostick_flight(t_int,x_0,q_des,robot)
    dl = angle2dl(q_des,robot);
    x_0(4) = dl;
    optns_with_event_f2s = odeset('RelTol',1e-3,'AbsTol',1e-3,'NormControl','on','Events',@(t,x)event_f2s(t,x,dl,robot));
    [t,x] = ode45( @(t,x)ode_pogostick_flight(t,x,robot),t_int,x_0,optns_with_event_f2s);
    dl_ar = x(:,4)';
    p3_ar = [x(:,1)' + robot.d/2*sin(x(:,3)' + q_des)
             x(:,2)' - robot.d/2*cos(x(:,3)' + q_des)
             zeros(1,length(t))];
    p4_ar = [x(:,1)' - robot.d/2*sin(x(:,3)' + q_des)
             x(:,2)' + robot.d/2*cos(x(:,3)' + q_des)
             zeros(1,length(t))];
end

