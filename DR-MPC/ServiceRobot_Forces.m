clear all
close all
global nx nu ny Ts L w b mean_train std_train Nobs P Q R

%% Initialization
Set_Params % Call Parameters

% Generate a Reference Trajectory
req_t = Required_Time(goal,0.5);
ts=timeseries(goal',req_t);
ref = resample(ts,0:Ts:req_t(end));
ref = ref.data';
ref(:,end+1:end+K) = repmat(goal(:,end),1,K);

load(['NN_Weights/Data_',str]); % Load Neural Network Parameters
load('obs_traj.mat'); % Load Obstacle States and Inputs

Create_Figure % Initialize the Figure

%% Generate a DR-MPC controller
if exist(['FORCESNLP',str])==0
   ServiceRobot_Forces_Gen;
end

%% Define a zero-mean GP prior with an RBF kernel
meanfunc = {@meanZero};
covfunc = {@covSEiso};
likfunc ={@likGauss};
hyp = struct('mean', [] , 'cov', [0 0], 'lik', -1);

%% Main DR-MPC Loop
t = 1;
x(:,1) = x0; % Set robot's initial state

ref_idx = 0;
cost=[];
terminate=0;

solve_time=tic; % Start recording computation time

while ~terminate

    % Render robot and its trajectory
    set(rob_traj, 'xdata', x(1,1:t),'ydata', x(2,1:t));
    set(rob_pl,'Position',[x(1,t)-rob_rad, x(2,t)-rob_rad, 2*rob_rad, 2*rob_rad]);

    GP_pred % Perform GPR to predict the obstacles' motion

    [current_ref, ref, ref_idx] = choose_ref(x, ref, t, K, ref_idx); % Choose the closest reference trajectory to follow
    set(fut_pl,'xdata',current_ref(1,:),'ydata',current_ref(2,:)); % Render the current reference trajectory

    obs_idx = choose_obs(obs_state, x, t, 4); % Choose the obstacles within the radius of 4

    % Use the GP predictions only for the chosen obstacles
    var_in = zeros(ny, ny, K-1, L);
    mu_in = zeros(ny, K-1, L);
    var_in(:,:,:,1:length(obs_idx)) = pos_var{t}(:,:,2:end-1,obs_idx);
    mu_in(:,:,1:length(obs_idx)) = pos_mu{t}(:,2:end-1,obs_idx);

    problem.xinit = x(:,t); % Set the initial state for DR-MPC 
    
    % Warm start the solver with previous DR-MPC predictions
    if t==1
        problem.x0 = zeros((nx + nu)*K,1);
        problem.x0(1:4) = x(:,t);
    else
        problem.x0=[reshape([U{t-1}(:,2:end);X_pred{t-1}(:,2:end)],(nx + nu)*(K-1),1); [U{t-1}(:,end);X_pred{t-1}(:,end)]];
    end

    % Set DR-MPC inputs
    problem.all_parameters = [current_ref(:,1); % Reference for k=0
                              reshape([ ...
                                    current_ref(:,2:K); ... % Reference for k=1:K+1
                                    reshape(permute(mu_in,[1 3 2]),[ny*L K-1]); ... % GP means
                                    reshape(permute(var_in,[1 2 4 3]),[ny*ny*L K-1]); ... % GP covariances
                                    repmat(statobs,1,K-1)], ... % Static obstacle positions
                              (ny+L*(ny+ny*ny)+Nobs*2)*(K-1),1)];
    
    % run the solver
    [sol,exitflag{t},info] = feval(['FORCESNLP',str],problem);  
    
    if exitflag{t}~=1
        warning('Some Problem')
    end
    fprintf('\nFORCES took %d iterations and %f seconds to solve the problem.\n',info.it,info.solvetime);


    U{t} = zeros(nu,K); % Contains all control inputs along the horizon
    X_pred{t} = zeros(nx,K); % Contains all predicted states along the horizon
    for i=1:K
        temp = sol.(['x',sprintf('%02d',i)]);
        X_pred{t}(:,i) = temp(nu+1:nu+nx,1);
        U{t}(:,i) = temp(1:nu,1);
    end   
    cost = [cost info.pobj]; % Total Cost vector


    % Apply controls to the robot

    x(1,t+1)=x(1,t)+Ts*x(3,t)+Ts^2/2*U{t}(1,1);
    x(2,t+1)=x(2,t)+Ts*x(4,t)+Ts^2/2*U{t}(2,1);
    x(3,t+1)=x(3,t)+Ts*U{t}(1,1);
    x(4,t+1)=x(4,t)+Ts*U{t}(2,1);

    % Render the predicted trajectory 
    set(rob_pr,'xdata',X_pred{t}(1,:),'ydata',X_pred{t}(2,:));
    drawnow

    % For recording
    if save_video==true
        frame = getframe(gcf);
        writeVideo(writerObj, frame);
    end

    % Terminate if the goal is reached or there is a collision
    if norm(x(1:2,t+1)-goal(:,end))<=0.05 || any(vecnorm(x(1:2,t+1)-obs_state(:,t+1,:)) < (rob_rad+obs_rad+safe_rad))
        % Render robot and its trajectory
        set(rob_traj, 'xdata', x(1,1:t+1),'ydata', x(2,1:t+1));
        set(rob_pl,'Position',[x(1,t+1)-rob_rad, x(2,t+1)-rob_rad, 2*rob_rad, 2*rob_rad]);
        
        % For recording
        if save_video==true
            frame = getframe(gcf);
            writeVideo(writerObj, frame);
        end

        terminate=1;
    end
    
    % Step increment
    t = t + 1;
end

% Total computational time
comp_time=toc(solve_time);

% Save the video
if save_video==true
    close(writerObj);    
end

save(['forces_',str])
