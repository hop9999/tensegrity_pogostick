function dxdt = ode_pogostick_stance(t,q,p3,robot)
    g = [0
        -9.8
         0];

    x = q(1);
    y = q(2);
    theta = q(3);
    CoM = [x;y;0];

    r = [robot.d/2*cos(theta)
         robot.d/2*sin(theta)
         0];

    p1 = CoM + r;
    p2 = CoM - r;
    
    dl = q(4);
    [p4, f14, f13, f24, f23] = static_stance(p1,p2,p3,dl,robot);
    moment = moment_massive_rod(theta,f14,f13,f24,f23,robot);
    dl_d = stance_control(q,moment,robot);
    
    pos_eq = (robot.m*g - f14 - f24 - f13 - f23)/robot.m;

    n = [pos_eq(1:2)
         moment];
    dxdt = zeros(7,1);
    
    dxdt(1:3) = q(5:7);
    dxdt(4) = dl_d;
    dxdt(5:7) = n;
end

