function [x4, f14, f13, f24, f23] = static_stance(x1,x2,x3,dl,robot)

    x0 = pi/2;
    
    [q,fval] = fminunc(@(q)energy_stance(q,x1,x2,x3,dl,robot),x0,robot.fminc_options);
    
    x4 = x3 + [robot.d*cos(q)
               robot.d*sin(q)
               0];
           
    f14 = (robot.k + robot.k_er(1))*(norm(x1 - x4) - robot.l0 - robot.l0_er(1) + dl)*(x1 - x4)/norm(x1 - x4);
    f24 = (robot.k + robot.k_er(2))*(norm(x2 - x4) - robot.l0 - robot.l0_er(2) - dl)*(x2 - x4)/norm(x2 - x4);
    f13 = (robot.k + robot.k_er(3))*(norm(x1 - x3) - robot.l0 - robot.l0_er(3) - dl)*(x1 - x3)/norm(x1 - x3);
    f23 = (robot.k + robot.k_er(4))*(norm(x2 - x3) - robot.l0 - robot.l0_er(4) + dl)*(x2 - x3)/norm(x2 - x3);
    
    if f14'*(x1 - x4) < 0
        f14 = [0
               0
               0];
    end
    if f24'*(x2 - x4) < 0
        f24 = [0
               0
               0];
    end
    if f13'*(x1 - x3) < 0
        f13 = [0
               0
               0];
    end
    if f23'*(x2 - x3) < 0
        f23 = [0
               0
               0];
    end

end