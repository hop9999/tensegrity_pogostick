function [position,isterminal,direction] = event_s2f(t,q,p3,robot)


    x = q(1);
    y = q(2);
    theta = q(3);
    dl = q(4);
    
    CoM = [x;y;0];

    r = [robot.d/2*cos(theta)
        robot.d/2*sin(theta)
        0];

    p1 = double(CoM + r);
    p2 = double(CoM - r);
      
    [p4, f14, f13, f24, f23] = static_stance(p1,p2,p3,dl,robot);
    fr = - f14 - f24 - f13 - f23;
    position = [fr(2),p1(2),p2(2)]; % The value that we want to be zero
    isterminal = [1,1,1];  % Halt integration 
    direction = [-1,0,0]; 
end