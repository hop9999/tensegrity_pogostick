function [position,isterminal,direction] = event_f2s(t,q,dl,robot)
    x = q(1);
    y = q(2);
    theta = q(3);
    CoM = [x;y;0];

    r = [robot.d/2*cos(theta)
        robot.d/2*sin(theta)
        0];
    q_des = dl2angle(dl,robot);
    
    p1 = double(CoM + r);
    p2 = double(CoM - r);
    
    y_p3 = y - robot.d/2*cos(theta + q_des);
    
    position = [y_p3,p1(2),p2(2)]; % The value that we want to be zero
    isterminal = [1,1,1];  % Halt integration 
    direction = [-1,0,0]; 
end