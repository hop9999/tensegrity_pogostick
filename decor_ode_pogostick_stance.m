function [t,x,p3_ar,p4_ar,fr_ar,dl_ar,mom_ar,robot] = decor_ode_pogostick_stance(t_int,x_0,p3,robot)
    optns_with_event_s2f = odeset('RelTol',1e-3,'AbsTol',1e-3,'NormControl','on','Events',@(t,x)event_s2f(t,x,p3,robot));
    [t,x] = ode45( @(t,x)ode_pogostick_stance(t,x,p3,robot),t_int,x_0,optns_with_event_s2f);
    
    p3_ar = [p3(1)*ones(1,length(t))
             p3(2)*ones(1,length(t))
             zeros(1,length(t))];
    p4_ar = zeros(3,length(t));
    fr_ar = zeros(4,length(t));
    mom_ar = zeros(1,length(t));
    dl_ar = x(:,4)';
    
    for j = 1:length(t)
        
        x_p = x(j,1);
        y_p = x(j,2);
        theta_p = x(j,3);
        CoM = [x_p;y_p;0];

        r = [robot.d/2*cos(theta_p)
             robot.d/2*sin(theta_p)
             0];

        p1 = CoM + r;
        p2 = CoM - r;
        [p4, f14, f13, f24, f23] = pogostick_static(p1,p2,p3,dl_ar(j),robot);
        moment = (skew(r)*(-f14) + skew(r)*(-f13) + skew(-r)*(-f24) + skew(-r)*(-f23))/robot.I;
        mom_ar(:,j) = moment(3);
        fr_ar(:,j) = [norm(f14); 
                      norm(f13); 
                      norm(f24); 
                      norm(f23)];
        p4_ar(:,j) = p4;
    end
end

