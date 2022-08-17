close all
global L W minAngle angIncrement maxAngle minVel minW maxW delta velIncrement maxVel Ts wIncrement root new_pl vels angles poscont w q_goal K r2 controller steercontrol t xy21 xy21_var;


% 0.00005 0.0001 0.0005 0.001 0.002 0.1 0.05 0.02 0.01
load('Simulations/sol_1e-05_11.mat')
veh_coords{1}=veh_coord;
Paths{1,:}=Path;
load('Simulations/sol_2e-05_11.mat')
veh_coords{2}=veh_coord;
Paths{2,:}=Path;
load('Simulations/sol_5e-05_11.mat')
veh_coords{3}=veh_coord;
Paths{3,:}=Path;
load('Simulations/sol_0.0001_11.mat')
veh_coords{4}=veh_coord;
Paths{4,:}=Path;
load('Simulations/sol_0.0005_11.mat')
veh_coords{5}=veh_coord;
Paths{5,:}=Path;
load('Simulations/sol_0.001_11.mat')
veh_coords{6}=veh_coord;
Paths{6,:}=Path;
load('Simulations/sol_0.002_11.mat')
veh_coords{7}=veh_coord;
Paths{7,:}=Path;
load('Simulations/sol_0.005_11.mat')
veh_coords{8}=veh_coord;
Paths{8,:}=Path;
load('Simulations/sol_0.01_11.mat')
veh_coords{9}=veh_coord;
Paths{9,:}=Path;
load('Simulations/sol_0.02_11.mat')
veh_coords{10}=veh_coord;
Paths{10,:}=Path;
load('Simulations/sol_0.05_11.mat')
veh_coords{11}=veh_coord;
Paths{11,:}=Path;
load('Simulations/sol_0.1_11.mat')
veh_coords{12}=veh_coord;
Paths{12,:}=Path;
load('../Scenario 2 RRT/Simulations/sol.mat')
veh_coords{13}=veh_coord;
Paths{13,:}=Path;
load('../Scenario 2 CC-RRT/Simulations/sol.mat')
veh_coords{14}=veh_coord;
Paths{14,:}=Path;

% legends={'1','2','3','4','5','6','7','8','9','10','11','12','13','14'};
% hold on
% for i=1:14
%     pl{i}=plot(veh_coords{i}(1,1:length(Paths{i})),veh_coords{i}(2,1:length(Paths{i})),'Color',rand(1,3));
% end
% legend(legends)




startstate=[21.85 0 pi/2]; %q_start
goalstate=[2 21.85 pi]; %q_goal
% obs=[3 2 0 0 1.5 0.8 1; 3 9 0 0 1.5 0.8 1]; %obstacle states, length, width, radius
% obs=[3 2 0 1.5 0.8 0.85; 1 9 0 1.5 0.8 0.85]; %obstacle states, length, width, radius
x_min=0; x_max=40; y_min=0; y_max=40; %limits
load('obs_traj4.mat') %Obstacle trajectories
L=2; W=2;
delta=0; %risk tolerance
w=0.1; %risk weight
K=10; %max depth
Ts=0.1; %Sampling time
% obs_state(:,1:end-3,2)=obs_state(:,4:end,2);
obs=[obs_state(:,1,1)';obs_state(:,1,2)'];
obs=obs(1,:);
obs_state=obs_state(:,:,1);
writerObj = VideoWriter(['Simulations/Final1.avi']);
writerObj.FrameRate = 10;
writerObj.Quality=100;
open(writerObj)

try
[status, msg, msgID] = mkdir('Figure_final1')
end

figure
set(gcf,'Color','white', 'Position',[197.6666666666667,67,664.6666666666666,540.6666666666666]);
set(gca,'Position',[0.073588709677419,0.083333333333333,0.884072580645161,0.884328358208955]);
axis([x_min x_max y_min y_max]);
hold on
xlabel('X')
ylabel('Y')
rectangle('Position',[0.05 0.05 39.95 39.95], 'FaceColor',[0.9 0.9 0.9],'EdgeColor','none')
rectangle('Position',[9.7 23.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[23.7 23.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[23.7 23.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[9.7 9.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[23.7 9.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)

rectangle('Position',[0.05 0.05 13 16.3],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[13 0.05 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[23.7 0.05 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[27 0.05 13 16.3],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[0.05 23.7 13 16.3],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[13 27 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[23.7 27 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[27 23.7 13 16.3],'FaceColor','w','EdgeColor','none') 

lane1=line([0.05 13],[16.3 16.3],'LineWidth',2,'Color','k');
lane2=line([0.05 13],[20 20],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane3=line([0.05 13],[23.7 23.7],'LineWidth',2,'Color','k');
lane4=line([27 39.95],[16.3 16.3],'LineWidth',2,'Color','k');
lane5=line([27 39.95],[20 20],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane6=line([27 39.95],[23.7 23.7],'LineWidth',2,'Color','k');
lane7=line([16.3 16.3],[0.05 13],'LineWidth',2,'Color','k');
lane8=line([20 20],[0.05 13],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane9=line([23.7 23.7],[0.05 13],'LineWidth',2,'Color','k');
lane10=line([16.3 16.3],[27 39.95],'LineWidth',2,'Color','k');
lane11=line([20 20],[27 39.95],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane12=line([23.7 23.7],[27 39.95],'LineWidth',2,'Color','k');


obstacle = [obs(:,1)-obs(:,end) obs(:,2)-obs(:,end) 2*obs(:,end) 2*obs(:,end)];
daspect([1 1 1])

hold on
plot(goalstate(1), goalstate(2),'*', 'Color', 'k', 'LineWidth',2);
plot(startstate(1), startstate(2), '*', 'Color', 'k', 'LineWidth',2);

color = {[0.00,0.45,0.74], [0.85,0.33,0.10], [0.93,0.69,0.13], [0.49,0.18,0.56], [0.47,0.67,0.19], [0.64,0.08,0.18], [0.00,0.45,0.74], [1.00,0.07,0.65], [0.62,0.66,0.58]};


plot(obs_state(1,:,1),obs_state(2,:,1),'Color','b','LineWidth',1,'LineStyle',':');
% plot(obs_state(1,:,2),obs_state(2,:,2),'Color',[150 150 150]/255,'LineStyle',':');

veh_traj_1=plot(startstate(1), startstate(2), 'Color', color{1} , 'LineWidth',2);
veh_pl_1=plot(startstate(1), startstate(2), '*', 'Color', 'k', 'LineWidth',2);
veh_traj_2=plot(startstate(1), startstate(2), 'Color', color{2}, 'LineWidth',2);
veh_pl_2=plot(startstate(1), startstate(2), '*', 'Color', 'k', 'LineWidth',2);
veh_traj_3=plot(startstate(1), startstate(2), 'Color', color{3}, 'LineWidth',2);
veh_pl_3=plot(startstate(1), startstate(2), '*', 'Color', 'k', 'LineWidth',2);
veh_traj_RRT=plot(startstate(1), startstate(2), 'Color', color{4}, 'LineWidth',2);
veh_pl_RRT=plot(startstate(1), startstate(2), '*', 'Color', 'k', 'LineWidth',2);
veh_traj_CC_RRT=plot(startstate(1), startstate(2), 'Color', color{5}, 'LineWidth',2);
veh_pl_CC_RRT=plot(startstate(1), startstate(2), '*', 'Color', 'k', 'LineWidth',2);
an1 = annotation('textbox',[0.146930731954901,0.543893580104637,0.101719366798244,0.066199980935679],'String',['$y_r^{goal}$'],'EdgeColor','none','Interpreter','latex','FontSize',16,'FitBoxToText','on')
an2 = annotation('textbox',[0.581683725876433,0.075382165076689,0.097364310333691,0.065844635545987],'String',['$y_r^{init}$'],'EdgeColor','none','Interpreter','latex','FontSize',16,'FitBoxToText','on')

meanfunc = {@meanZero};
covfunc = {@covSEiso};
likfunc ={@likGauss};
hyp = struct('mean', [] , 'cov', [0 0], 'lik', -1);
legend([veh_traj_1 veh_traj_2 veh_traj_3 veh_traj_RRT veh_traj_CC_RRT],'DR-RRT*$\,(\theta\!=\!10^{-4})$','DR-RRT*$\,(\theta\!=\!10^{-3})$','DR-RRT*$\,(\theta\!=\!5\!\times\! 10^{-3})$','RRT*','CC-RRT*','Interpreter','latex','Orientation','vertical','Location','northeast','FontSize',11,'Color','none','EdgeColor','none', 'Position',[0.571857041447519,0.769553390323376,0.321720276852766,0.188539738575764])
%legend([veh_traj_2 veh_traj_RRT veh_traj_CC_RRT],'DR-RRT*','RRT*','CC-RRT*','Interpreter','latex','Orientation','vertical','Location','northeast','FontSize',14,'Color','none','EdgeColor','none', 'Position',[0.562828449648024,0.819007330833109,0.316700196531478,0.133456558749328])

t=0;
while t<=max(length(Paths{3}),max(length(Paths{14}),length(Paths{4})))+1
    t=t+1;
    GP_pred1;
    if t>length(Paths{3})
        veh_coords{3}(1:2,t)=goalstate(1:2)';
    end
    if t>length(Paths{1})
        veh_coords{1}(1:2,t)=goalstate(1:2)';
    end
    if t>length(Paths{4})
        veh_coords{4}(1:2,t)=goalstate(1:2)';
    end
    if t>length(Paths{9})
        veh_coords{9}(1:2,t)=veh_coords{9}(1:2,length(Paths{9}));
    end
    if t>length(Paths{5})
        veh_coords{5}(1:2,t)=veh_coords{5}(1:2,length(Paths{5}));
    end
    if t==max(length(Paths{3}),max(length(Paths{14}),length(Paths{4})))
        veh_coords{4}(1:2,t)=goalstate(1:2)';
        veh_coords{1}(1:2,t)=goalstate(1:2)';
        veh_coords{3}(1:2,t)=goalstate(1:2)';
    end
    set(veh_pl_1,'xdata',veh_coords{3}(1,t));
    set(veh_pl_1,'ydata',veh_coords{3}(2,t));
    set(veh_traj_1, 'xdata', veh_coords{3}(1,1:t));
    set(veh_traj_1, 'ydata', veh_coords{3}(2,1:t));
    set(veh_pl_2,'xdata',veh_coords{1}(1,t));
    set(veh_pl_2,'ydata',veh_coords{1}(2,t));
    set(veh_traj_2, 'xdata', veh_coords{1}(1,1:t));
    set(veh_traj_2, 'ydata', veh_coords{1}(2,1:t));
    set(veh_pl_3,'xdata',veh_coords{4}(1,t));
    set(veh_pl_3,'ydata',veh_coords{4}(2,t));
    set(veh_traj_3, 'xdata', veh_coords{4}(1,1:t));
    set(veh_traj_3, 'ydata', veh_coords{4}(2,1:t));
    set(veh_pl_RRT,'xdata',veh_coords{9}(1,t));
    set(veh_pl_RRT,'ydata',veh_coords{9}(2,t));
    set(veh_traj_RRT, 'xdata', veh_coords{9}(1,1:t));
    set(veh_traj_RRT, 'ydata', veh_coords{9}(2,1:t));
    set(veh_pl_CC_RRT,'xdata',veh_coords{5}(1,t));
    set(veh_pl_CC_RRT,'ydata',veh_coords{5}(2,t));
    set(veh_traj_CC_RRT, 'xdata', veh_coords{5}(1,1:t));
    set(veh_traj_CC_RRT, 'ydata', veh_coords{5}(2,1:t));
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
    set(gcf,'InvertHardCopy','off');
    saveas(gcf,['Figure_final1/t2_',num2str(t)],'png');
    saveas(gcf,['Figure_final1/t2_',num2str(t)],'epsc');
    uistack(veh_traj_1,'top')
    uistack(veh_pl_1,'top')
    uistack(veh_traj_2,'top')
    uistack(veh_pl_2,'top')
    uistack(veh_traj_3,'top')
    uistack(veh_pl_3,'top')
    uistack(veh_traj_RRT,'top')
    uistack(veh_pl_RRT,'top')
    uistack(veh_traj_CC_RRT,'top')
    uistack(veh_pl_CC_RRT,'top')    
end
    
close(writerObj)
