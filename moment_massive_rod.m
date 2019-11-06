function moment = moment_massive_rod(theta,f14,f13,f24,f23,robot)

    r = [robot.d/2*cos(theta)
        robot.d/2*sin(theta)
        0];
    r_err = [robot.CoM_er*cos(theta)
            robot.CoM_er*sin(theta)
            0];
    
    moment_vec = (skew(r+r_err)*(-f14) + skew(r+r_err)*(-f13) + skew(-r+r_err)*(-f24) + skew(-r+r_err)*(-f23))/robot.I;
    moment = moment_vec(3);
end
