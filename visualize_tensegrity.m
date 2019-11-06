function visualize_tensegrity(robot,q,p3_array,p4_array,fr_array)
fig = figure;
fig.Name = "animation";
filename = 'testAnimated.gif';

    for i = 1:30:length(q)
        clf
        q(i,:)
        x = q(i,1);
        y = q(i,2);
        theta = q(i,3);
        CoM = [x;y;0];
        
        r = [robot.d/2*cos(theta)
            robot.d/2*sin(theta)
            0];
        fr = fr_array(:,i)/200;
        p1 = CoM + r
        p2 = CoM - r
        p3 = p3_array(:,i)
        p4 = p4_array(:,i);
        hold on
        
        plot([-3, 3],[0, 0],'k','LineWidth',0.5)
        scatter(p1(1),p1(2),10,'k','filled')
        scatter(p2(1),p2(2),10,'k','filled')
        p = plot([p1(1), p2(1)],[p1(2), p2(2)],'k','LineWidth',2);
        scatter(p3(1),p3(2),10,'k','filled')
        scatter(p4(1),p4(2),10,'k','filled')
        plot([p3(1), p4(1)],[p3(2), p4(2)],'k','LineWidth',2)
        plot([p3(1), p3(1) + fr(1)],[p3(2),  p3(2) + fr(2)],'r','LineWidth',2)
        axis equal
        xlim([-1 1])
        ylim([-0.3 1.7])
        pause(1e-2)
        frame = getframe(fig);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        del = 1e-3;
        % Write to the GIF File
        if i == 1
            imwrite(imind,cm,filename,'gif','Loopcount',inf,'DelayTime',del);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',del);
        end
        
    end

end