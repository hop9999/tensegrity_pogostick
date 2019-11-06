function P = energy_flight(q,x1,x2,dl,robot)
    x3 = [q(1)
          q(2)
          0];
    phi = q(3);
     
    x4 = x3 + [robot.d*cos(phi)
               robot.d*sin(phi)
               0];
           
    P14 = (robot.k + robot.k_er(1))*(norm(x1 - x4) - robot.l0 - robot.l0_er(1) + dl)^2;
    P24 = (robot.k + robot.k_er(2))*(norm(x2 - x4) - robot.l0 - robot.l0_er(2) - dl)^2;
    P13 = (robot.k + robot.k_er(3))*(norm(x1 - x3) - robot.l0 - robot.l0_er(3) + dl)^2;
    P23 = (robot.k + robot.k_er(4))*(norm(x2 - x3) - robot.l0 - robot.l0_er(4) - dl)^2;
    
    P = P14 + P24 + P13 + P23;
end
