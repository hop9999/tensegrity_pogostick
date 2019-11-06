function P = energy_stance(q,X1,X2,X3,dl,robot)

    X4 = X3 + [robot.d*cos(q)
               robot.d*sin(q)
               0];
           
    P14 = (robot.k + robot.k_er(1))*(norm(X1 - X4) - robot.l0 - robot.l0_er(1) + dl)^2;
    P24 = (robot.k + robot.k_er(2))*(norm(X2 - X4) - robot.l0 - robot.l0_er(2) - dl)^2;
    P = (P14 + P24);
end
