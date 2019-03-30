% figure(3);
% for plot_i=1:1:7
%     plot(vol(:,plot_i),'LineWidth',2);grid on;hold on;
% end
% xlabel('t (s)','FontSize',12,'FontWeight','bold','Color','k');
% ylabel('vol (rad/s)','FontSize',12,'FontWeight','bold','Color','k');
% legend('Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Joint7');
% 
% figure(4);
% for plot_i=1:1:7
%     plot(acc(:,plot_i),'LineWidth',2);grid on;hold on;
% end
% xlabel('t (s)','FontSize',12,'FontWeight','bold','Color','k');
% ylabel('acc (rad/s^2)','FontSize',12,'FontWeight','bold','Color','k');
% legend('Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Joint7');
% 
% 
% figure(5);
% for plot_i=1:1:7
%     plot(theta_traj(:,plot_i),'LineWidth',2);grid on;hold on;
% end
% xlabel('t (s)','FontSize',12,'FontWeight','bold','Color','k');
% ylabel('theta(rad)','FontSize',12,'FontWeight','bold','Color','k');
% legend('Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Joint7');
% figure(6);
% for plot_i=1:1:7
%     subplot(3,1,1);plot(q(plot_i,:),'LineWidth',2);grid on;hold on;
%     xlabel('t','FontSize',12,'FontWeight','bold','Color','k');
%     ylabel('theta','FontSize',12,'FontWeight','bold','Color','k');
%     subplot(3,1,2);plot(vv(plot_i,:),'LineWidth',2);grid on;hold on;
%     xlabel('t','FontSize',12,'FontWeight','bold','Color','k');
%     ylabel('vol','FontSize',12,'FontWeight','bold','Color','k');
%     subplot(3,1,3);plot(a(plot_i,:),'LineWidth',2);grid on;hold on;
%     xlabel('t','FontSize',12,'FontWeight','bold','Color','k');
%     ylabel('acc','FontSize',12,'FontWeight','bold','Color','k');
% end
% legend('Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Joint7');


% figure(7);
% for plot_i=1:1:7
%     subplot(3,1,1);plot(qq(plot_i,:),'LineWidth',2);grid on;hold on;
%     xlabel('t','FontSize',12,'FontWeight','bold','Color','k');
%     ylabel('theta','FontSize',12,'FontWeight','bold','Color','k');
%     subplot(3,1,2);plot(vvv(plot_i,:),'LineWidth',2);grid on;hold on;
%     xlabel('t','FontSize',12,'FontWeight','bold','Color','k');
%     ylabel('vol','FontSize',12,'FontWeight','bold','Color','k');
%     subplot(3,1,3);plot(aa(plot_i,:),'LineWidth',2);grid on;hold on;
%     xlabel('t','FontSize',12,'FontWeight','bold','Color','k');
%     ylabel('acc','FontSize',12,'FontWeight','bold','Color','k');
% end
% legend('Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Joint7');

figure(7);
for plot_i=1:1:7
    subplot(3,1,1);plot(theta_traj(:,plot_i),'LineWidth',2);grid on;hold on;
    xlabel('t','FontSize',12,'FontWeight','bold','Color','k');
    ylabel('theta','FontSize',12,'FontWeight','bold','Color','k');
    subplot(3,1,2);plot(vol(:,plot_i),'LineWidth',2);grid on;hold on;
    xlabel('t','FontSize',12,'FontWeight','bold','Color','k');
    ylabel('vol','FontSize',12,'FontWeight','bold','Color','k');
    subplot(3,1,3);plot(acc(:,plot_i),'LineWidth',2);grid on;hold on;
    xlabel('t','FontSize',12,'FontWeight','bold','Color','k');
    ylabel('acc','FontSize',12,'FontWeight','bold','Color','k');
end
legend('Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Joint7');
