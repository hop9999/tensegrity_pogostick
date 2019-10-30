function dl = angle2dl(q,robot)
    q = q/2 + pi/4;
    dl = (tan(q)*(robot.l0 - robot.d*cos(q)) - robot.l0 + robot.d*sin(q))/(tan(q) + 1);
end
