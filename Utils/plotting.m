function plotting(t,y,ref,e,tau)
    figure('position',[349.8,354.6,964.8,334.4])
    subplot(1,3,1)
    plot(t,y(:,[1,3]),'LineWidth',2); hold on; grid on
    plot(t,ref,'k--','LineWidth',1)
    title('Link Response','FontSize',12);
    xlabel('Time (s)','FontSize',11);
    ylabel('Position (rad)','FontSize',11);
    ylim([-0.2,1.8])
    legend('q_1(t)','q_2(t)','Ref');

    subplot(1,3,2)
    plot(t,e,'LineWidth',2); hold on;
    plot(t,zeros(size(t)),'k--','LineWidth',2); grid on
    title('Tracking Error','FontSize',12);
    xlabel('Time (s)','FontSize',11);
    ylabel('Error (rad)','FontSize',11);
    legend('e_1(t)','e_2(t)');

    subplot(1,3,3)
    plot(t,tau,'LineWidth',2); grid on
    title('Joint Torques','FontSize',12);
    xlabel('Time (s)','FontSize',11);
    ylabel('Torques (rad)','FontSize',11);
    ylim([-12,12])
    legend('u_1(t)','u_2(t)') 
end