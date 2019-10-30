function q_des = flight_control(q,robot)
    theta = q(3);
    theta_d = q(7);
    x_d = q(5);
    x_d_des = 0.0;
    ts = 0.14;
    tf = 0.7;
    kx = 0.008;
    (x_d_des - x_d)
    xf = -x_d*ts/2 + kx*(x_d_des - x_d)
    q_des = -(theta + tf*theta_d) - asin(xf/(robot.d/1.7));
end
