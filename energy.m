function P = energy(q,X1,X2,X3,dl,robot)

    X4 = X3 + [robot.d*cos(q)
               robot.d*sin(q)
               0];
           
    P14 = robot.k*(norm(X1 - X4) - robot.l0 + dl)^2;
    P24 = robot.k*(norm(X2 - X4) - robot.l0 - dl)^2;
    P = (P14 + P24);
end
