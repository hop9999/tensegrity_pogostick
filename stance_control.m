function dl = stance_control(q,robot)
    theta = q(3);
    theta_d = q(6);
    kq = 1.5;
    kq_d = 0.5;
    dl = kq*(0 - theta) + kq_d*(0 - theta_d)

    if robot.dl + dl > robot.dl_max
        dl = robot.dl_max - robot.dl;
    end
    if robot.dl + dl < -robot.dl_max
        dl = -robot.dl_max - robot.dl;
    end
    if robot.dl > robot.dl_max
        dl = 0;
    end
    if robot.dl < -robot.dl_max
        dl = 0;
    end
end
