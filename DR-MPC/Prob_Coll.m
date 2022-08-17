clear all
clc

strlistprob = {'forces_0_00001.mat','forces_0_0001.mat','forces_0_01.mat', 'nocol_W1_N_100.mat', 'nocol_forces_CC_MPC.mat'};
for ilist=1:length(strlistprob)
    load(strlistprob{ilist})
    clearvars -except x0 Xout xy21 xy21_var cost Path strlistprob veh_coord Path ilist obs_mu obs_std cost_traj
    veh_coord{ilist}=x0;
    Path{ilist}=Xout;
    obs_mu{ilist}=xy21;
    obs_std{ilist}=xy21_var;
    cost_traj{ilist} = sum(cost);
end
K=10;
r=0.2;
L=3;

% for l=1:length(strlistprob)
l=1;
for l1=1:length(strlistprob)
    if length(Path{l1}) > length(Path{l})
        l = l1;
    end
end
for i=1:length(Path{l})
    for j=1:L
        for k=1:K
        obs_rand{i}{j}(:,:,k)=mvnrnd(obs_mu{l}{i}(:,k,j),obs_std{l}{i}(:,:,k,j).^2,50000);
        end
    end
end
% end

for l=1:length(strlistprob)
    for i=1:length(Path{l})
        for j=1:L
            for k=1:K
                dist{l}{i}{j}(:,k)=vecnorm(obs_rand{i}{j}(:,:,k)'-Path{l}{i}(1:2,k))';
                prob_col{l}(i,k,j)=mean(dist{l}{i}{j}(:,k)<r);
            end
        end
    end
    Path_prob_col{l}=max(max(prob_col{l},[],2),[],3);
    Traj_prob_col(l)=mean(Path_prob_col{l});
end

load(strlistprob{1}) %Obstacle trajectories
for l=1:length(strlistprob)
    for i=1:length(Path{l})
        for j=1:L
            for k=1:K
                dist_a{l}{i}{j}(k)=vecnorm(obs_state(1:2,i+k-1,j)-Path{l}{i}(1:2,k))';
                prob_col_a{l}(i,k,j)=(dist_a{l}{i}{j}(k)<r);
            end
        end
    end
    Path_prob_col_a{l}=max(max(prob_col_a{l},[],2),[],3);
    Traj_prob_col_a(l)=mean(Path_prob_col_a{l});
end

figure(1)
for l=1:length(strlistprob)
plot(Path_prob_col{l},'LineWidth',1)
hold on
end
title('Predicted Probability of Collision over time $t$','interpreter','latex','FontSize',14)
legend('$0.00001$','$0.0001$','$0.01$','SAA-MPC','CC-MPC','interpreter','latex','location','northwest','FontSize',12)

figure(2)
for l=1:length(strlistprob)
plot(Path_prob_col_a{l},'LineWidth',1)
hold on
end
title('Actual Probability of Collision over time $t$','interpreter','latex','FontSize',14)
legend('$0.00001$','$0.0001$','$0.01$','SAA-MPC','CC-MPC','interpreter','latex','location','northwest','FontSize',12)
