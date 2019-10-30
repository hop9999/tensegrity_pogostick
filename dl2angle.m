function q = dl2angle(dl,robot)
    alpha = atan((-dl - robot.l0)/(dl - robot.l0));
    q = 2*alpha - pi/2;
end
