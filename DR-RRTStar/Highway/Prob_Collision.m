clear all
clc

load('Simulations/sol_0.01.mat')
veh_coord_0_01=veh_coord;
Path_0_01=Path;
obs_mu_0_01=xy21;
obs_std_0_01=xy21_var;
load('Simulations/sol_0.05.mat')
veh_coord_0_05=veh_coord;
Path_0_05=Path;
obs_mu_0_05=xy21;
obs_std_0_05=xy21_var;
load('Simulations/sol_0.0001.mat')
veh_coord_0_0001=veh_coord;
Path_0_0001=Path;
obs_mu_0_0001=xy21;
obs_std_0_0001=xy21_var;
load('Simulations/sol_0.1.mat')
veh_coord_0_1=veh_coord;
Path_0_1=Path;
obs_mu_0_1=xy21;
obs_std_0_1=xy21_var;
K=11;
r=2;

for i=1:length(Path_0_01)
    for j=1:2
        for k=1:K
        obs_rand_0_01{i}{j}(:,:,k)=mvnrnd(obs_mu_0_01{i}{j}(k,:)',obs_std_0_01{i}{j}(:,:,k).^2,50000);
        end
    end
end

for i=1:length(Path_0_0001)
    for j=1:2
        for k=1:K
        obs_rand_0_0001{i}{j}(:,:,k)=mvnrnd(obs_mu_0_0001{i}{j}(k,:)',obs_std_0_0001{i}{j}(:,:,k).^2,50000);
        end
    end
end

for i=1:length(Path_0_05)
    for j=1:2
        for k=1:K
        obs_rand_0_05{i}{j}(:,:,k)=mvnrnd(obs_mu_0_05{i}{j}(k,:)',obs_std_0_05{i}{j}(:,:,k).^2,50000);
        end
    end
end

for i=1:length(Path_0_1)
    for j=1:2
        for k=1:K
        obs_rand_0_1{i}{j}(:,:,k)=mvnrnd(obs_mu_0_1{i}{j}(k,:)',obs_std_0_1{i}{j}(:,:,k).^2,50000);
        end
    end
end

for i=1:length(Path_0_01)
    for j=1:2
        for k=1:length(Path_0_01{i})
        dist_0_01{i}{j}(:,k)=vecnorm(obs_rand_0_01{i}{j}(:,:,k)'-Path_0_01{i}(k).coord(1:2))';
        prob_col_0_01(i,k,j)=mean(dist_0_01{i}{j}(:,k)<r);
        end
    end
end

for i=1:length(Path_0_0001)
    for j=1:2
        for k=1:length(Path_0_0001{i})
        dist_0_0001{i}{j}(:,k)=vecnorm(obs_rand_0_0001{i}{j}(:,:,k)'-Path_0_0001{i}(k).coord(1:2))';
        prob_col_0_0001(i,k,j)=mean(dist_0_0001{i}{j}(:,k)<r);
        end
    end
end

for i=1:length(Path_0_05)
    for j=1:2
        for k=1:length(Path_0_05{i})
        dist_0_05{i}{j}(:,k)=vecnorm(obs_rand_0_05{i}{j}(:,:,k)'-Path_0_05{i}(k).coord(1:2))';
        prob_col_0_05(i,k,j)=mean(dist_0_05{i}{j}(:,k)<r);
        end
    end
end

for i=1:length(Path_0_1)
    for j=1:2
        for k=1:length(Path_0_1{i})
        dist_0_1{i}{j}(:,k)=vecnorm(obs_rand_0_1{i}{j}(:,:,k)'-Path_0_1{i}(k).coord(1:2))';
        prob_col_0_1(i,k,j)=mean(dist_0_1{i}{j}(:,k)<r);
        end
    end
end

Path_prob_col_0_0001=max(max(prob_col_0_0001,[],2),[],3);
Path_prob_col_0_01=max(max(prob_col_0_01,[],2),[],3);
Path_prob_col_0_05=max(max(prob_col_0_05,[],2),[],3);
Path_prob_col_0_1=max(max(prob_col_0_1,[],2),[],3);
Traj_prob_col_0_0001=mean(Path_prob_col_0_0001);
Traj_prob_col_0_01=mean(Path_prob_col_0_01);
Traj_prob_col_0_05=mean(Path_prob_col_0_05);
Traj_prob_col_0_1=mean(Path_prob_col_0_1);

cost_0_0001=0;
for i=1:length(Path_0_0001)
cost_0_0001 = cost_0_0001+Path_0_0001{i}(end).cost;
end

cost_0_01=0;
for i=1:length(Path_0_01)
cost_0_01 = cost_0_01+Path_0_01{i}(end).cost;
end

cost_0_05=0;
for i=1:length(Path_0_05)
cost_0_05 = cost_0_05+Path_0_05{i}(end).cost;
end

cost_0_1=0;
for i=1:length(Path_0_1)
cost_0_1 = cost_0_1+Path_0_1{i}(end).cost;
end

load('obs_traj1.mat') %Obstacle trajectories
for i=1:length(Path_0_01)
for j=1:2
for k=1:length(Path_0_01{i})
dist_a_0_01{i}{j}(k)=vecnorm(obs_state(1:2,i+k-1,j)-Path_0_01{i}(k).coord(1:2))';
prob_col_a_0_01(i,k,j)=(dist_a_0_01{i}{j}(k)<r);
end
end
end
for i=1:length(Path_0_0001)
for j=1:2
for k=1:length(Path_0_0001{i})
dist_a_0_0001{i}{j}(k)=vecnorm(obs_state(1:2,i+k-1,j)-Path_0_0001{i}(k).coord(1:2))';
prob_col_a_0_0001(i,k,j)=(dist_a_0_0001{i}{j}(k)<r);
end
end
end
for i=1:length(Path_0_05)
for j=1:2
for k=1:length(Path_0_05{i})
dist_a_0_05{i}{j}(k)=vecnorm(obs_state(1:2,i+k-1,j)-Path_0_05{i}(k).coord(1:2))';
prob_col_a_0_05(i,k,j)=(dist_a_0_05{i}{j}(k)<r);
end
end
end
for i=1:length(Path_0_1)
for j=1:2
for k=1:length(Path_0_1{i})
dist_a_0_1{i}{j}(k)=vecnorm(obs_state(1:2,i+k-1,j)-Path_0_1{i}(k).coord(1:2))';
prob_col_a_0_1(i,k,j)=(dist_a_0_1{i}{j}(k)<r);
end
end
end
Path_prob_col_a_0_0001=max(max(prob_col_a_0_0001,[],2),[],3);
Path_prob_col_a_0_01=max(max(prob_col_a_0_01,[],2),[],3);
Path_prob_col_a_0_05=max(max(prob_col_a_0_05,[],2),[],3);
Path_prob_col_a_0_1=max(max(prob_col_a_0_1,[],2),[],3);
Traj_prob_col_a_0_0001=mean(Path_prob_col_a_0_0001);
Traj_prob_col_a_0_01=mean(Path_prob_col_a_0_01);
Traj_prob_col_a_0_05=mean(Path_prob_col_a_0_05);
Traj_prob_col_a_0_1=mean(Path_prob_col_a_0_1);



figure(1)
hold on
plot(Path_prob_col_0_0001,'LineWidth',1)
plot(Path_prob_col_0_01,'LineWidth',1)
plot(Path_prob_col_0_05,'LineWidth',1)
plot(Path_prob_col_0_1,'LineWidth',1)
title('Predicted Probability of Collision over time $t$','interpreter','latex','FontSize',14)
legend('$0.0001$','$0.01$','$0.05$','$0.1$','interpreter','latex','location','northwest','FontSize',12)

figure(2)
hold on
plot(Path_prob_col_a_0_0001,'LineWidth',1)
plot(Path_prob_col_a_0_01,'LineWidth',1)
plot(Path_prob_col_a_0_05,'LineWidth',1)
plot(Path_prob_col_a_0_1,'LineWidth',1)
title('Predicted Probability of Collision over time $t$','interpreter','latex','FontSize',14)
legend('$0.0001$','$0.01$','$0.05$','$0.1$','interpreter','latex','location','northwest','FontSize',12)