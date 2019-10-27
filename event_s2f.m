function [position,isterminal,direction] = event_s2f(t,q,p3,robot)


    x = q(1);
    y = q(2);
    theta = q(3);
    CoM = [x;y;0];

    r = [robot.d/2*cos(theta)
        robot.d/2*sin(theta)
        0];

    p1 = double(CoM + r);
    p2 = double(CoM - r);

      
    [theta_stick, f14, f13, f24, f23] = pogostick_static(p1,p2,p3,robot);
    fr = - f14 - f24 - f13 - f23;
    f = double(fr(2));
    position = [fr(2),p1(2),p2(2)]; % The value that we want to be zero
    isterminal = [1,1,1];  % Halt integration 
    direction = [-1,0,0]; 
end