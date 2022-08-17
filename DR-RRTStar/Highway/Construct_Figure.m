figure
% set(gcf,'Position',[0,300,1035,345]);
% set(gca,'Position',[0.052434456928839,0.137956331862895,0.924157303370789,0.785723427417197]);
set(gcf,'Position',[-1923,453,1813,335]);
set(gcf,'Color','white');
axis([x_min x_max y_min y_max]);
hold on
xlabel('X', 'FontSize',16)
ylabel('Y', 'FontSize',16)
set(gca,'Position',[0.034215311131506,0.158208955223881,0.954606680584542,0.716417910447761]);
set(gca,'Color',[0.9 0.9 0.9], 'FontSize',14);
lane1=line([0 x_max],[0 0],'LineWidth',2,'Color','k');
lane2=line([0 x_max],[3.7 3.7],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane3=line([0 x_max],[7.4 7.4],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane4=line([0 x_max],[y_max y_max],'LineWidth',2,'Color','k');

obstacle = [obs(:,1)-obs(:,end) obs(:,2)-obs(:,end) 2*obs(:,end) 2*obs(:,end)];
veh_coord=zeros(nx,K1);
veh_coord(:,1)=startstate';
daspect([1 1 1])

% for j=1:size(obs,1)
% Gp(:,:,j)=[cos(obs(j,3)) sin(obs(j,3));cos(obs(j,3)+pi) sin(obs(j,3)+pi);cos(obs(j,3)+pi/2) sin(obs(j,3)+pi/2); cos(obs(j,3)-pi/2) sin(obs(j,3)-pi/2)];
% g_p(1,j)=Gp(1,:,j)*obs(j,1:2)'+obs(j,end-2)/2;
% g_p(2,j)=Gp(2,:,j)*obs(j,1:2)'+obs(j,end-2)/2;
% g_p(3,j)=Gp(3,:,j)*obs(j,1:2)'+obs(j,end-1)/2;
% g_p(4,j)=Gp(4,:,j)*obs(j,1:2)'+obs(j,end-1)/2;
% end
% 
% Gp_veh=[cos(veh_coord(3,1)) sin(veh_coord(3,1));cos(veh_coord(3,1)+pi) sin(veh_coord(3,1)+pi);cos(veh_coord(3,1)+pi/2) sin(veh_coord(3,1)+pi/2); cos(veh_coord(3,1)-pi/2) sin(veh_coord(3,1)-pi/2)];
% g_p_veh(1)=Gp_veh(1,:)*veh_coord(1:2,1)+L/2;
% g_p_veh(2)=Gp_veh(2,:)*veh_coord(1:2,1)+L/2;
% g_p_veh(3)=Gp_veh(3,:)*veh_coord(1:2,1)+W/2;
% g_p_veh(4)=Gp_veh(4,:)*veh_coord(1:2,1)+W/2;
% movobs_veh(1,:,1)=(inv(Gp_veh([1 3],:,1))*g_p_veh([1 3])')';
% movobs_veh(2,:,1)=(inv(Gp_veh([1 4],:,1))*g_p_veh([1 4])')';
% movobs_veh(3,:,1)=(inv(Gp_veh([2 4],:,1))*g_p_veh([2 4])')';
% movobs_veh(4,:,1)=(inv(Gp_veh([2 3],:,1))*g_p_veh([2 3])')';  

hold on

an1 = annotation('textbox',[0.930030995459863,0.50717023801752,0.049207405181119,0.135713935657742],'String',['$y_r^{goal}$'],'EdgeColor','none','Interpreter','latex','FontSize',22,'FitBoxToText','on')
an2 = annotation('textbox',[0.046597212309403,0.504973729614894,0.047012060646699,0.134925369433503],'String',['$y_r^{init}$'],'EdgeColor','none','Interpreter','latex','FontSize',22,'FitBoxToText','on')

plot(goalstate(1), goalstate(2),'.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);
plot(startstate(1), startstate(2), '.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);

plot(obs_state(1,:,1),obs_state(2,:,1),'Color','b','LineStyle',':', 'LineWidth', 1);
plot(obs_state(1,:,2),obs_state(2,:,2),'Color','b','LineStyle',':', 'LineWidth', 1);


% mov_veh=fill(movobs_veh(:,1,1),movobs_veh(:,2,1),[205 16 26]/255,'EdgeColor',[164 13 20]/255,'FaceAlpha',0.8);
% mov_veh=rectangle('Position',[veh_coord(1,1)-W_r veh_coord(2,1)-W_r 2*W_r 2*W_r],'Curvature',[1 1],'FaceColor',[[205 16 26]/255 0.8],'EdgeColor',[[164 13 20]/255 0.8]);
%plot(q_goal.coord(1), q_goal.coord(2),'*', 'Color', 'k', 'LineWidth',2);
%plot(q_start.coord(1), q_start.coord(2), '*', 'Color', 'k', 'LineWidth',2);
new_pl{1}=plot(q_start.coord(1), q_start.coord(2), '.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);

veh_traj=plot(q_start.coord(1), q_start.coord(2), 'Color', 'k', 'LineWidth',2);
veh_pl=plot(q_start.coord(1), q_start.coord(2), '.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);

