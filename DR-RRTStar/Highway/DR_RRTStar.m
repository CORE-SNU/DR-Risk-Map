% % % for t=1:180
% % % for i=1:2000
% % % if rand()<=0.1
% % %     q_rand_all{t}(:,i)=q_goal;
% % % else
% % %     q_rand_all{t}(:,i) = [x_min+(x_max-x_min)*rand() y_min+(y_max-y_min)*rand() 2*pi*rand()]';
% % % end
% % % end
% % % end
clear all
close all
Comp_Time=[]; Final_Cost=[]; Trajs=[];
thetat=[0.00001 0.0001 0.001 0.005 0.01 0.05 0.1];
for theta=thetat
% theta=0.001;
global theta ucost L W_r W_o W_s minAngle angIncrement maxAngle minVel minW maxW delta velIncrement maxVel Ts wIncrement root new_pl vels angles poscont w_risk q_goal K r2 controller steercontrol t xy21 xy21_var;

load('obs_traj1.mat') %Obstacle trajectories
startstate=[1 5.5 0]; %q_start
goalstate=[75 5.55 0]; %q_goal
% obs=[3 2 0 0 1.5 0.8 1; 3 9 0 0 1.5 0.8 1]; %obstacle states, length, width, radius
% obs=[3 2 0 1.5 0.8 0.85; 1 9 0 1.5 0.8 0.85]; %obstacle states, length, width, radius
x_min=0; x_max=80; y_min=0; y_max=11.1; %limits
r_rrt=1; %RRT* radius
tryGoalProbability=0.1; %goal is chosen as q_rand with probability tryGoalProability
w_risk=0.05; %risk weight
K=10; %max depth
Ts=0.1; %Sampling time
time_to_grow=1;
K1=100; %Max traj
nx=3;

% theta=0.001;
alpha=0.95;

%Vehicle parameters
L=2; W_r = 1; W_o = 1; W_s = 0.1;
minAngle=-0.6; maxAngle=0.6; angIncrement=0.01; minVel=1; maxVel=10; velIncrement=0.1; wIncrement=0.02; minW=tan(minAngle)*maxVel/L; maxW=tan(maxAngle)*maxVel/L;
% obs_state(:,1:end-3,2)=obs_state(:,4:end,2);
% obs_state(end-1,:,1)=W;
% obs_state(end-1,:,2)=W;
% obs_state(end-2,:,1)=L;
% obs_state(end-2,:,2)=L;
obs=[obs_state(:,1,1)';obs_state(:,1,2)'];

r2=([W_r+W_o+W_s W_r+W_o+W_s]).^2; %safety radius squared
delta=r2(1)*3/100; %risk tolerance
vels=minVel:velIncrement:maxVel;
% angles=minAngle:angIncrement:maxAngle;
ws=minW:wIncrement:maxW;
poscont=combvec(vels,ws);
ucost = 0.00001*sum(poscont.*poscont);
load('q_rand.mat')
load('Dynamics.mat')
% obs_state(:,:,2)=repmat(obs_state(:,1,2),1,120);
try
[status, msg, msgID] = mkdir(['Figure_',num2str(theta)])
[status, msg, msgID] = mkdir('Simulations/');
end

Initialize_tree
Construct_Figure
onlySDP
% onlySteer
t=0;
i=0;
terminate=0;
solve_time=tic;
try
while ~terminate
    t=t+1;
    Observe_and_Update;
    GP_pred_NN_1;
    Construct_Safe_Tree;
    Grow
    Plan_Path
    Path{t}=Best;
    Tree{t}=nodes_to_grow;
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
% % % %     set(gcf,'InvertHardCopy','off');
% % % %     saveas(gcf,['Figure_',num2str(theta),'/t_',num2str(t)],'png');
% % % %     saveas(gcf,['Figure_',num2str(theta),'/t_',num2str(t)],'epsc');
    veh_coord(:,t+1)=Best(2).coord;
%     Gp_veh=[cos(veh_coord(3,t+1)) sin(veh_coord(3,t+1));cos(veh_coord(3,t+1)+pi) sin(veh_coord(3,t+1)+pi);cos(veh_coord(3,t+1)+pi/2) sin(veh_coord(3,t+1)+pi/2); cos(veh_coord(3,t+1)-pi/2) sin(veh_coord(3,t+1)-pi/2)];
%     g_p_veh(1)=Gp_veh(1,:)*veh_coord(1:2,t+1)+L/2;
%     g_p_veh(2)=Gp_veh(2,:)*veh_coord(1:2,t+1)+L/2;
%     g_p_veh(3)=Gp_veh(3,:)*veh_coord(1:2,t+1)+W/2;
%     g_p_veh(4)=Gp_veh(4,:)*veh_coord(1:2,t+1)+W/2;
%     movobs_veh(1,:,t+1)=(inv(Gp_veh([1 3],:,1))*g_p_veh([1 3])')';
%     movobs_veh(2,:,t+1)=(inv(Gp_veh([1 4],:,1))*g_p_veh([1 4])')';
%     movobs_veh(3,:,t+1)=(inv(Gp_veh([2 4],:,1))*g_p_veh([2 4])')';
%     movobs_veh(4,:,t+1)=(inv(Gp_veh([2 3],:,1))*g_p_veh([2 3])')';  
% 
%     uistack(mov_veh,'top')
    uistack(veh_traj,'top')
    uistack(veh_pl,'top')
%     set(mov_veh,'xdata',movobs_veh(:,1,t+1));
%     set(mov_veh,'ydata',movobs_veh(:,2,t+1));
    set(mov_veh,'Position',[veh_coord(1,t+1)-W_r veh_coord(2,t+1)-W_r 2*W_r 2*W_r]);
    set(veh_pl,'xdata',Best(2).coord(1));
    set(veh_pl,'ydata',Best(2).coord(2));
    set(veh_traj, 'xdata', veh_coord(1,1:t+1));
    set(veh_traj, 'ydata', veh_coord(2,1:t+1));
    try
        delete(best_pl)
    end
    if norm(veh_coord(1:2,t+1)-q_goal.coord(1:2))<=0.5
        terminate=1;
    end
end
close(writerObj)
save(['Simulations/sol_',num2str(theta),'.mat'],'Path','Tree','veh_coord', 'xy21','xy21_var','xytheta_mu','xytheta_var');
cost=0;
for i=1:length(Path)
    cost=cost+Path{i}(end).cost;
end
Final_Cost = [Final_Cost cost]; 
Comp_Time= [Comp_Time toc(solve_time)];
Trajs = [Trajs {veh_coord(1:2,:)}];
catch
    try
close(writerObj)
save(['Simulations/sol_',num2str(theta),'.mat'],'Path','Tree','veh_coord', 'xy21','xy21_var','xytheta_mu1','xytheta_var1');
    catch
        continue
    end
cost=0;
for i=1:length(Path)
    cost=cost+Path{i}(end).cost;
end
Final_Cost = [Final_Cost cost]; 
Comp_Time= [Comp_Time toc(solve_time)];
Trajs = [Trajs {veh_coord(1:2,:)}];
end
clearvars -except theta thetat Comp_Time Final_Cost Trajs
end