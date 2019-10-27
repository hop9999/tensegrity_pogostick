function dxdt = ode_pogostick_stance(t,q,p3,robot)
    
    g = [0
        -9.8
         0];

    x = q(1);
    y = q(2);
    theta = q(3);
    robot.dl = robot.dl + stance_control(q,robot);
    CoM = [x;y;0];

    dxdt = zeros(6,1);

    r = [robot.d/2*cos(theta)
         robot.d/2*sin(theta)
         0];

    p1 = CoM + r;
    p2 = CoM - r;

    [p4, f14, f13, f24, f23] = pogostick_static(p1,p2,p3,robot);
    
    pos_eq = (robot.m*g - f14 - f24 - f13 - f23)/robot.m;
    orient_eq = (skew(r)*(-f14) + skew(r)*(-f13) + skew(-r)*(-f24) + skew(-r)*(-f23))/robot.I;
    n = [pos_eq(1:2)
         orient_eq(3)];

    dxdt(1:3) = q(4:6);
    dxdt(4:6) = n;
end

