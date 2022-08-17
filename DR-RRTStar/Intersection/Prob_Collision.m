clear all
clc

load('Simulations/sol_5e-05_11.mat')
veh_coords_0_0001=veh_coord;
Path_0_0001=Path;
obs_mu_0_0001=xy21;
obs_std_0_0001=xy21_var;
load('Simulations/sol_1e-05_11.mat')
veh_coords_0_001=veh_coord;
Path_0_001=Path;
obs_mu_0_001=xy21;
obs_std_0_001=xy21_var;
load('Simulations/sol_0.0001_11.mat')
veh_coords_0_005=veh_coord;
Path_0_005=Path;
obs_mu_0_005=xy21;
obs_std_0_005=xy21_var;
load('Simulations/sol_0.01_11.mat')
veh_coords_RRT=veh_coord;
Path_RRT=Path;
obs_mu_RRT=xy21;
obs_std_RRT=xy21_var;
load('Simulations/sol_0.0005_11.mat')
veh_coords_CC_RRT=veh_coord;
Path_CC_RRT=Path;
obs_mu_CC_RRT=xy21;
obs_std_CC_RRT=xy21_var;

K=11;
r=2;

for i=1:length(Path_0_0001)
    for k=1:K
        obs_rand_0_0001{i}(:,:,k)=mvnrnd(obs_mu_0_0001{i}{1}(k,:)',obs_std_0_0001{i}{1}(:,:,k).^2,50000);
    end
end

for i=1:length(Path_0_001)
    for k=1:K
        obs_rand_0_001{i}(:,:,k)=mvnrnd(obs_mu_0_001{i}{1}(k,:)',obs_std_0_001{i}{1}(:,:,k).^2,50000);
    end
end

for i=1:length(Path_0_005)
    for k=1:K
        obs_rand_0_005{i}(:,:,k)=mvnrnd(obs_mu_0_005{i}{1}(k,:)',obs_std_0_005{i}{1}(:,:,k).^2,50000);
    end
end

for i=1:length(Path_RRT)
    for k=1:K
        obs_rand_RRT{i}(:,:,k)=mvnrnd(obs_mu_RRT{i}{1}(k,:)',obs_std_RRT{i}{1}(:,:,k).^2,50000);
    end
end

for i=1:length(Path_CC_RRT)
    for k=1:K
        obs_rand_CC_RRT{i}(:,:,k)=mvnrnd(obs_mu_CC_RRT{i}{1}(k,:)',obs_std_CC_RRT{i}{1}(:,:,k).^2,50000);
    end
end


for i=1:length(Path_0_0001)
    for k=1:length(Path_0_0001{i})
        dist_0_0001{i}(:,k)=vecnorm(obs_rand_0_0001{i}(:,:,k)'-Path_0_0001{i}(k).coord(1:2))';
        prob_col_0_0001(i,k)=mean(dist_0_0001{i}(:,k)<r);
    end
end

for i=1:length(Path_0_001)
    for k=1:length(Path_0_001{i})
        dist_0_001{i}(:,k)=vecnorm(obs_rand_0_001{i}(:,:,k)'-Path_0_001{i}(k).coord(1:2))';
        prob_col_0_001(i,k)=mean(dist_0_001{i}(:,k)<r);
    end
end
for i=1:length(Path_0_005)
    for k=1:length(Path_0_005{i})
        dist_0_005{i}(:,k)=vecnorm(obs_rand_0_005{i}(:,:,k)'-Path_0_005{i}(k).coord(1:2))';
        prob_col_0_005(i,k)=mean(dist_0_005{i}(:,k)<r);
    end
end

for i=1:length(Path_RRT)
    for k=1:length(Path_RRT{i})
        dist_RRT{i}(:,k)=vecnorm(obs_rand_RRT{i}(:,:,k)'-Path_RRT{i}(k).coord(1:2))';
        prob_col_RRT(i,k)=mean(dist_RRT{i}(:,k)<r);
    end
end

for i=1:length(Path_CC_RRT)
    for k=1:length(Path_CC_RRT{i})
        dist_CC_RRT{i}(:,k)=vecnorm(obs_rand_CC_RRT{i}(:,:,k)'-Path_CC_RRT{i}(k).coord(1:2))';
        prob_col_CC_RRT(i,k)=mean(dist_CC_RRT{i}(:,k)<r);
    end
end


Path_prob_col_0_0001=max(prob_col_0_0001,[],2);
Path_prob_col_0_001=max(prob_col_0_001,[],2);
Path_prob_col_0_005=max(prob_col_0_005,[],2);
Path_prob_col_RRT=max(prob_col_RRT,[],2);
Path_prob_col_CC_RRT=max(prob_col_CC_RRT,[],2);
Traj_prob_col_0_0001=mean(Path_prob_col_0_0001);
Traj_prob_col_0_001=mean(Path_prob_col_0_001);
Traj_prob_col_0_005=mean(Path_prob_col_0_005);
Traj_prob_col_RRT=mean(Path_prob_col_RRT);
Traj_prob_col_CC_RRT=mean(Path_prob_col_CC_RRT);

figure(1)
P1=plot(0:0.1:(length(Path_0_0001)*0.1-0.1),Path_prob_col_0_0001,'LineWidth',1);
hold on
P2=plot(0:0.1:(length(Path_0_001)*0.1-0.1),Path_prob_col_0_001,'LineWidth',1);
P3=plot(0:0.1:(length(Path_0_005)*0.1-0.1),Path_prob_col_0_005,'LineWidth',1);
P4=plot(0:0.1:(length(Path_RRT)*0.1-0.1),Path_prob_col_RRT,'LineWidth',1);
P5=plot(0:0.1:(length(Path_CC_RRT)*0.1-0.1),Path_prob_col_CC_RRT,'LineWidth',1);
legend([P1 P2 P3 P4 P5],'$\;\theta=0.0001$','$\;\theta=0.001$','$\;\theta=0.005$','RRT*','CC-RRT*','Interpreter','latex','FontSize',12,'Color','none','EdgeColor','none')

cost_0_0001=0;
for i=1:length(Path_0_0001)
cost_0_0001 = cost_0_0001+Path_0_0001{i}(end).cost;
end

cost_0_001=0;
for i=1:length(Path_0_001)
cost_0_001 = cost_0_001+Path_0_001{i}(end).cost;
end

cost_0_005=0;
for i=1:length(Path_0_005)
cost_0_005 = cost_0_005+Path_0_005{i}(end).cost;
end

cost_RRT=0;
for i=1:length(Path_RRT)
cost_RRT = cost_RRT+Path_RRT{i}(end).cost;
end

cost_CC_RRT=0;
for i=1:length(Path_CC_RRT)
cost_CC_RRT = cost_CC_RRT+Path_CC_RRT{i}(end).cost;
end




load('obs_traj4.mat') %Obstacle trajectories
for i=1:length(Path_0_0001)
    for k=1:length(Path_0_0001{i})
        dist_a_0_0001{i}(k)=vecnorm(obs_state(1:2,i+k-1)-Path_0_0001{i}(k).coord(1:2))';
        prob_col_a_0_0001(i,k)=mean(dist_a_0_0001{i}(k)<r);
    end
end

for i=1:length(Path_0_001)
    for k=1:length(Path_0_001{i})
        dist_a_0_001{i}(k)=vecnorm(obs_state(1:2,i+k-1)-Path_0_001{i}(k).coord(1:2))';
        prob_col_a_0_001(i,k)=mean(dist_a_0_001{i}(k)<r);
    end
end
for i=1:length(Path_0_005)
    for k=1:length(Path_0_005{i})
        dist_a_0_005{i}(k)=vecnorm(obs_state(1:2,i+k-1)-Path_0_005{i}(k).coord(1:2))';
        prob_col_a_0_005(i,k)=mean(dist_a_0_005{i}(k)<r);
    end
end

for i=1:length(Path_RRT)
    for k=1:length(Path_RRT{i})
        dist_a_RRT{i}(k)=vecnorm(obs_state(1:2,i+k-1)-Path_RRT{i}(k).coord(1:2))';
        prob_col_a_RRT(i,k)=mean(dist_a_RRT{i}(k)<r);
    end
end

for i=1:length(Path_CC_RRT)
    for k=1:length(Path_CC_RRT{i})
        dist_a_CC_RRT{i}(k)=vecnorm(obs_state(1:2,i+k-1)-Path_CC_RRT{i}(k).coord(1:2))';
        prob_col_a_CC_RRT(i,k)=mean(dist_a_CC_RRT{i}(k)<r);
    end
end


Path_prob_col_a_0_0001=max(max(prob_col_a_0_0001,[],2),[],3);
Path_prob_col_a_0_001=max(max(prob_col_a_0_001,[],2),[],3);
Path_prob_col_a_0_005=max(max(prob_col_a_0_005,[],2),[],3);
Path_prob_col_a_RRT=max(max(prob_col_a_RRT,[],2),[],3);
Path_prob_col_a_CC_RRT=max(max(prob_col_CC_RRT,[],2),[],3);
Traj_prob_col_a_0_0001=mean(Path_prob_col_a_0_0001);
Traj_prob_col_a_0_001=mean(Path_prob_col_a_0_001);
Traj_prob_col_a_0_005=mean(Path_prob_col_a_0_005);
Traj_prob_col_a_RRT=mean(Path_prob_col_a_RRT);
Traj_prob_col_a_CC_RRT=mean(Path_prob_col_a_CC_RRT);