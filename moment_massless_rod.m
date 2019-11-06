function P = moment_massless_rod(q,x1,x2,x3,dl,robot)
    r = [robot.d*cos(q)
         robot.d*sin(q)
         0];
     
    x4 = x3 + r;
           
    f14 = (robot.k + robot.k_er(1))*(norm(x1 - x4) - robot.l0 - robot.l0_er(1) + dl)*(x1 - x4)/norm(x1 - x4);
    f24 = (robot.k + robot.k_er(2))*(norm(x2 - x4) - robot.l0 - robot.l0_er(2) - dl)*(x2 - x4)/norm(x2 - x4);
    moment = skew(r)*(f14) + skew(r)*(f24);
    P = moment(3)^2;
end
