%% Initialize GP datasets 
if t==1
    x_tr = num2cell(permute(obs_state(1:obs_nx,t,:),[3 1 2]),2);
    for j=1:obs_nu
        y_tr{j} = num2cell(permute(obs_input(j,t,:),[3 2 1])+normrnd(0,0.0001),2);
    end
end   

%% Predict each obstacles' position
for obs_idx=1:size(obs_state,3) 

        % Initialize GP prediction to current obstacle position with zero covariance
        xt_mu = obs_state(1:obs_nx,t,obs_idx); % mu_x^{t,0} = x_o(t)
        xt_var = zeros(obs_nx,obs_nx,1); % Sigma_x^{t,0} = 0

        % Extracts mu_y^{t,0} and Sigma_y^{t,0} from state predictions
        pos_mu{t}(:,1,obs_idx) = xt_mu(1:2,1);
        pos_var{t}(:,:,1,obs_idx) = diag([xt_var(1,1,1) xt_var(2,2,1)]);
        
        ut_mu = zeros(K, obs_nu);
        u_var = zeros(obs_nu, obs_nu, K);
        for k=1:K
            M=size(x_tr{obs_idx},1);
            if k==1         
                for j=1:obs_nu
                    % In each time step t, first optimize the GP
                    % hyperparameters for each input dimension based on the current dataset
                    hyp_{j} = minimize(hyp, @gp, -2000, @infGaussLik, meanfunc, covfunc, likfunc, x_tr{obs_idx}, y_tr{j}{obs_idx}); % For u_o[1]
                    
                    % Learned input covariance matrix K_j(hat{x}, hat{x})
                    Kxx{j} = covSEiso(hyp_{j}(1).cov,x_tr{obs_idx},x_tr{obs_idx});
                    
                    % Learned diagonal length scale matrix L_j
                    Lf{j} = exp(-2*hyp_{j}.cov(1));
                    
                    % Learned observation noise variance σ^2_{n,j}
                    sigman{j} = exp(hyp_{j}.lik);
                    
                    % Cholesky decomposition of GP prior (K_j (hat{x}, hat{x}) + σ^2_{v,j} I)
                    % (Small regularization term is added for numerical stability)
                    W{j} = chol(Kxx{j}+sigman{j}^2*eye(M,M)+10^(-6)*eye(M,M));
                end
            end
            
            
            for j=1:obs_nu
                
                kxx{j} = covSEiso(hyp_{j}.cov,xt_mu(:,k)',xt_mu(:,k)'); % K_j (mu_x^{t,k}, mu_x^{t,k})
                kXx{j} = covSEiso(hyp_{j}.cov,x_tr{obs_idx},xt_mu(:,k)'); % K_j (hat{x}, mu_x^{t,k})
    
                v{j} = W{j}'\kXx{j};
                beta{j} = W{j}\(W{j}'\(y_tr{j}{obs_idx}));       
    
                % Mean vector mu_u^{t,k}
                ut_mu(k,j) = kXx{j}'*beta{j}; 
    
                % Derivative of mu_u^j evaluated at mu_x^{t,k}
                dmu_u_dx(j,:,k) = ((kXx{j}.*((x_tr{obs_idx}-xt_mu(:,k)')*Lf{j}))'*beta{j})';
    
                % Covariance matrix Sigma_u^{t,k}  
                u_var(j,j,k) = kxx{j}-v{j}'*v{j}+dmu_u_dx(j,:,k)*(xt_var(:,:,k))*dmu_u_dx(j,:,k)';
            end

            % Covariance matrix Sigma_{xu}^{t,k}
            xu_var(:,:,k) = xt_var(:,:,k)*dmu_u_dx(:,:,k)';

            % Propagate the means through the obstacle dynamics
            % The outputs are predicted mean mu_x^{t,k+1} at next time stage
            % and partial derivatives of the dynamics with respect to the states and controls
            [xt_mu(:,k+1), dmu_x_dx, dmu_x_du] = vehicledyn(ut_mu(k,:),xt_mu(:,k)');

            % Predicted covariance matrix Sigma_x^{t,k+1}
            xt_var(:,:,k+1)=dmu_x_dx*xt_var(:,:,k)*dmu_x_dx'+dmu_x_du*u_var(:,:,k)*dmu_x_du'+dmu_x_dx*xu_var(:,:,k)*dmu_x_du'+dmu_x_du*xu_var(:,:,k)'*dmu_x_dx';

            % Extracts mu_y^{t,k+1} and Sigma_y^{t,k+1} from state predictions
            pos_mu{t}(:,k+1,obs_idx) = xt_mu(1:2,k+1);
            pos_var{t}(:,:,k+1,obs_idx) = diag([xt_var(1,1,k+1) xt_var(2,2,k+1)]);
        end

        % Render obstacle predictions
        for k=1:K+1
            if k~=1
                try
                    set(obs_obj{obs_idx}(k),'Position',[pos_mu{t}(1,k,obs_idx)-obs_rad pos_mu{t}(2,k,obs_idx)-obs_rad 2*obs_rad 2*obs_rad]);
                catch
                    obs_obj{obs_idx}(k)=rectangle('Position',[pos_mu{t}(1,k,obs_idx)-obs_rad pos_mu{t}(2,k,obs_idx)-obs_rad 2*obs_rad 2*obs_rad],'FaceColor',[0 .5 .5 0.05],'EdgeColor',[0 .5 .5],'Curvature',[1 1]);
                end
            end
        end     
        try
            set(obs_obj{obs_idx}(1),'Position',[pos_mu{t}(1,1,obs_idx)-obs_rad pos_mu{t}(2,1,obs_idx)-obs_rad 2*obs_rad 2*obs_rad]);
        catch
            obs_obj{obs_idx}(1)=rectangle('Position',[pos_mu{t}(1,1,obs_idx)-obs_rad pos_mu{t}(2,1,obs_idx)-obs_rad 2*obs_rad 2*obs_rad],'FaceColor',[0 .5 .5 1],'EdgeColor',[0 .5 .5],'Curvature',[1 1]);
        end
        drawnow
end
    
%% Update GP datasets
for obs_idx=1:size(obs_state,3)
    x_tr{obs_idx}=[x_tr{obs_idx}; obs_state(1:obs_nx,t,obs_idx)'];
    for j=1:obs_nu
        y_tr{j}{obs_idx}=[y_tr{j}{obs_idx};  obs_input(j,t,obs_idx)'+normrnd(0,0.0001)];
    end
end

%% Keep only the latest M_max observations
if size(x_tr{1},1) >= M_max
    for obsj=1:size(obs_state,3)
        x_tr{obsj}=x_tr{obsj}(end-M_max+1:end,:);
        for j=1:obs_nu
            y_tr{j}{obsj}=y_tr{j}{obsj}(end-M_max+1:end,:);
        end
    end
end       