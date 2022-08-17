% for t=1:180
% for i=1:2000
% if rand()<=0.1
%     q_rand_all{t}(:,i)=q_goal.coord;
% else
%     if rand()<=0.5
%     q_rand_all{t}(:,i) = [x_min+(x_max/2-x_min)*rand() y_max/2+(3.7)*rand() 2*pi*rand()]';
%     else
%     q_rand_all{t}(:,i) = [x_max/2+3.7*rand() y_min+(y_max/2+3.7-y_min)*rand() 2*pi*rand()]';
%     end
% end
% end
% end
% for t=1:180
% for i=1:2000
% if rand()<=0.1
%     q_rand_all{t}(:,i)=q_goal.coord;
% else
%      rand1 = [x_min+(x_max-x_min)*rand() y_min+(y_max-y_min)*rand() 2*pi*rand()]';
%      if (rand1(1)>0 && rand1(1)<13 && rand1(2)>16.3 && rand1(2)<23.7) || (rand1(1)>27 && rand1(1)<40 && rand1(2)>16.3 && rand1(2)<23.7) || ...
%         (rand1(1)>16.3 && rand1(1)<23.7 && rand1(2)>0 && rand1(2)<13) || (rand1(1)>16.3 && rand1(1)<23.7 && rand1(2)>27 && rand1(2)<40) || ...
%         (rand1(1)>13 && rand1(1)<27 && rand1(2)>13 && rand1(2)<27 && norm(rand1(1:2)-[13;27])>3.3 && norm(rand1(1:2)-[13;13])>3.3 && norm(rand1(1:2)-[27;27])>3.3 && norm(rand1(1:2)-[27;13])>3.3)
%         t1=true;
%      else
%         t1=false;
%      end
%      while t1==false
%         rand1 = [x_min+(x_max-x_min)*rand() y_min+(y_max-y_min)*rand() 2*pi*rand()]';
%         if (rand1(1)>0 && rand1(1)<13 && rand1(2)>16.3 && rand1(2)<23.7) || (rand1(1)>27 && rand1(1)<40 && rand1(2)>16.3 && rand1(2)<23.7) || ...
%             (rand1(1)>16.3 && rand1(1)<23.7 && rand1(2)>0 && rand1(2)<13) || (rand1(1)>16.3 && rand1(1)<23.7 && rand1(2)>27 && rand1(2)<40) || ...
%             (rand1(1)>13 && rand1(1)<27 && rand1(2)>13 && rand1(2)<27 && norm(rand1(1:2)-[13;27])>3.3 && norm(rand1(1:2)-[13;13])>3.3 && norm(rand1(1:2)-[27;27])>3.3 && norm(rand1(1:2)-[27;13])>3.3)
%             t1=true;
%         else
%             t1=false;
%         end
%      end 
%      q_rand_all{t}(:,i)=rand1;
% end
% end
% end

clear all
close all
%0.00005 0.0001 0.0005 0.001 0.002 0.01 0.02
thetat=[0.00001 0.00005 0.0001 0.0005 0.001 0.002 0.005 0.01 0.02 0.05 0.1];
thetat = 0.00005;
for theta=thetat
global L W minAngle angIncrement maxAngle minVel minW maxW delta velIncrement maxVel Ts wIncrement root new_pl vels angles poscont w q_goal K r2 controller steercontrol t xy21 xy21_var;

load('obs_traj4.mat') %Obstacle trajectories
% obs_state(1:3,:,:)=zeros(3,101,2);
startstate=[21.85 0 pi/2]; %q_start
goalstate=[2 21.85 pi]; %q_goal
% obs=[3 2 0 0 1.5 0.8 1; 3 9 0 0 1.5 0.8 1]; %obstacle states, length, width, radius
% obs=[3 2 0 1.5 0.8 0.85; 1 9 0 1.5 0.8 0.85]; %obstacle states, length, width, radius
x_min=0; x_max=40; y_min=0; y_max=40; %limits
r_rrt=1; %RRT* radius
tryGoalProbability=0.1; %goal is chosen as q_rand with probability tryGoalProability
delta=0; %risk tolerance
w=0; %risk weight
K=10; %max depth
Ts=0.1; %Sampling time
time_to_grow=5;
K1=100; %Max traj
nx=3;

% theta=0.001;
alpha=0.95;

%Vehicle parameters
L=2; W=2;
minAngle=-0.6; maxAngle=0.6; angIncrement=0.01; minVel=0; maxVel=10; velIncrement=0.1; wIncrement=0.02; minW=tan(minAngle)*maxVel/W; maxW=tan(maxAngle)*maxVel/W;
% obs_state(:,1:end-3,2)=obs_state(:,4:end,2);
obs=[obs_state(:,1,1)';obs_state(:,1,2)'];
obs=obs(1,:);
obs_state=obs_state(:,:,1);
r2=([W W]).^2; %safety radius squared
vels=minVel:velIncrement:maxVel;
% angles=minAngle:angIncrement:maxAngle;
ws=minW:wIncrement:maxW;
poscont=combvec(vels,ws);

load('q_rand.mat')

% obs_state(:,:,2)=repmat(obs_state(:,1,2),1,120);
try
[status, msg, msgID] = mkdir(['Figure_',num2str(theta),'_new']);
[status, msg, msgID] = mkdir('Simulations/');
end

Initialize_tree
Construct_Figure1
onlySDP
% onlySteer
t=0;
i=0;
terminate=0;
try
while ~terminate
    t=t+1;
%     set(mov_veh,'Position',[veh_coord(1,t)-W/2 veh_coord(2,t)-W/2 W W]);
    set(veh_traj, 'xdata', veh_coord(1,1:t));
    set(veh_traj, 'ydata', veh_coord(2,1:t));
    Observe_and_Update;
    GP_pred1;
    Construct_Safe_Tree;
    Grow
    if size(nodes_to_grow)==1
        time_to_grow=10;
        Grow;
        time_to_grow=3;
    end
    Plan_Path
    Path{t}=Best;
    Tree{t}=nodes_to_grow;
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
    set(gcf,'InvertHardCopy','off');
    saveas(gcf,['Figure_',num2str(theta),'_new/t_',num2str(t)],'png');
    saveas(gcf,['Figure_',num2str(theta),'_new/t_',num2str(t)],'epsc');
    veh_coord(:,t+1)=Best(2).coord;
% % %     Gp_veh=[cos(veh_coord(3,t+1)) sin(veh_coord(3,t+1));cos(veh_coord(3,t+1)+pi) sin(veh_coord(3,t+1)+pi);cos(veh_coord(3,t+1)+pi/2) sin(veh_coord(3,t+1)+pi/2); cos(veh_coord(3,t+1)-pi/2) sin(veh_coord(3,t+1)-pi/2)];
% % %     g_p_veh(1)=Gp_veh(1,:)*veh_coord(1:2,t+1)+L/2;
% % %     g_p_veh(2)=Gp_veh(2,:)*veh_coord(1:2,t+1)+L/2;
% % %     g_p_veh(3)=Gp_veh(3,:)*veh_coord(1:2,t+1)+W/2;
% % %     g_p_veh(4)=Gp_veh(4,:)*veh_coord(1:2,t+1)+W/2;
% % %     movobs_veh(1,:,t+1)=(inv(Gp_veh([1 3],:,1))*g_p_veh([1 3])')';
% % %     movobs_veh(2,:,t+1)=(inv(Gp_veh([1 4],:,1))*g_p_veh([1 4])')';
% % %     movobs_veh(3,:,t+1)=(inv(Gp_veh([2 4],:,1))*g_p_veh([2 4])')';
% % %     movobs_veh(4,:,t+1)=(inv(Gp_veh([2 3],:,1))*g_p_veh([2 3])')'; 
%     R=[cos(veh_coord(3,t+1)) -sin(veh_coord(3,t+1));sin(veh_coord(3,t+1)) cos(veh_coord(3,t+1))];
%     movobs_veh1(:,:,t+1)=(veh_coord(1:2,t+1)+R*[L/2 L/2 -L/2 -L/2;W/2 -W/2 -W/2 W/2])';
%     uistack(mov_veh,'top')
    uistack(veh_traj,'top')
    uistack(veh_pl,'top')
% % %     set(mov_veh,'xdata',movobs_veh(:,1,t+1));
% % %     set(mov_veh,'ydata',movobs_veh(:,2,t+1));
    set(veh_pl,'xdata',Best(2).coord(1));
    set(veh_pl,'ydata',Best(2).coord(2));
    try
        delete(best_pl)
    end
    if norm(veh_coord(1:2,t+1)-q_goal.coord(1:2))<=0.5 || norm(veh_coord(1:2,t)-obs_state(1:2,t,1))<=W 
        %|| norm(veh_coord(1:2,t)-obs_state(1:2,t,2))<=W
        terminate=1;
    end
end
close(writerObj)
save(['Simulations/sol_',num2str(theta),'_',num2str(11),'.mat'],'Path','Tree','veh_coord','xy21','xy21_var','xytheta_mu','xytheta_var');
catch
    try
close(writerObj)
save(['Simulations/sol_',num2str(theta),'_',num2str(11),'.mat'],'Path','Tree','veh_coord', 'xy21','xy21_var','xytheta_mu','xytheta_var');
    catch
        continue
    end
end
clearvars -except theta thetat
end