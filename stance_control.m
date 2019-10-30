function dl_d = stance_control(q,moment,robot)
    dl = q(4);
    theta = q(3);
    theta_d = q(7);
    kq = 50;
    kq_d = 100;
    km = 1;% - 10*abs(theta);
    dl_d = kq*(0 - theta) + kq_d*(0 - theta_d) + km*(0 - moment);
    
    if dl_d > robot.dl_d_max
        dl_d = robot.dl_d_max;
    end
    if dl_d < -robot.dl_d_max
        dl_d = -robot.dl_d_max;
    end
    if dl > robot.dl_max
        dl_d = 0;
    end
    if dl < -robot.dl_max
        dl_d = 0;
    end
end
