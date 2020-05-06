function visualization(t,y,ref,e,tau,file_name,export_video)
    global l1 l2
    
    frame_rate = 30;
    
    %% Interpolation for linear time
    tspan = t(end) - t(1);
    t_lin = linspace(t(1), t(end), tspan*frame_rate);
    
    q1_lin = interp1(t,y(:,1),t_lin,'spline')';
    q2_lin = interp1(t,y(:,3),t_lin,'spline')';
    [p1,p2] = RR_EndEffector(l1,l2,q1_lin,q2_lin);
    
    q_ref_lin = interp1(t,ref,t_lin,'spline')';
    [p1_ref, p2_ref] = RR_EndEffector(l1,l2,q_ref_lin,q_ref_lin);
    
    e_lin = interp1(t,e,t_lin,'spline')';
    tau_lin = interp1(t,tau,t_lin,'spline')';
    
    %% plotting
    fig = figure('position',[200,50,1150,750]);
%     fig = figure('position',get(0,'ScreenSize'));
    
    plotting(t,y,ref,e,tau,4);
    
    subplot(3,4,4);
    ph_q1 = plot(t_lin(1),q1_lin(1),'bo','MarkerFaceColor','blue');
    ph_q2 = plot(t_lin(1),q2_lin(1),'ro','MarkerFaceColor','red');
    legend('q_1(t)','q_2(t)','Ref');
    
    subplot(3,4,8);
    ph_e1 = plot(t_lin(1),e_lin(1,1),'bo','MarkerFaceColor','blue');
    ph_e2 = plot(t_lin(1),e_lin(2,1),'ro','MarkerFaceColor','red');
    legend off
    
    subplot(3,4,12);
    ph_tau1 = plot(t_lin(1),tau_lin(1,1),'bo','MarkerFaceColor','blue');
    ph_tau2 = plot(t_lin(1),tau_lin(2,1),'ro','MarkerFaceColor','red');
    legend off
    
    if export_video
        movieVector(length(t_lin)) = getframe;
    end
    
    for i = 1:length(t_lin)
        ti = t_lin(i);
        
        subplot(3,4,[1,11]);
        % plot the links
        plot([0 p1(i,1)], [0 p1(i,2)], 'b','LineWidth',2); 
        hold on;
        plot([p1(i,1),p2(i,1)], [p1(i,2),p2(i,2)], 'r','LineWidth',2);
        % plot the reference
        plot([0 p1_ref(i,1)], [0 p1_ref(i,2)], 'b--','LineWidth',2);
        plot([p1_ref(i,1),p2_ref(i,1)], [p1_ref(i,2),p2_ref(i,2)], 'r--','LineWidth',2);
        hold off;
        
        title(['RR with controller ',file_name,' and ref Rect.     t=',num2str(t_lin(i))]);
        xlim([-0.5,0.5]);
        ylim([-0.5,0.5]);
        
        ph_q1.XData = ti; ph_q1.YData = q1_lin(i);
        ph_q2.XData = ti; ph_q2.YData = q2_lin(i);
        ph_e1.XData = ti; ph_e1.YData = e_lin(1,i);
        ph_e2.XData = ti; ph_e2.YData = e_lin(2,i);
        ph_tau1.XData = ti; ph_tau1.YData = tau_lin(1,i);
        ph_tau2.XData = ti; ph_tau2.YData = tau_lin(2,i);
        
    %     pause(1.0/frame_rate*2)
        if export_video
            movieVector(i) = getframe(fig);
        end
    end

    if export_video
%         myVW = VideoWriter('animation\RR_PDcontrol','MPEG-4');
        myVW = VideoWriter(['animation\',file_name],'MPEG-4');
        myVW.FrameRate = frame_rate/2;

        open(myVW);
        writeVideo(myVW,movieVector);
        close(myVW);

        disp(['Video export at ', file_name,' Done!!']);
    end
end 