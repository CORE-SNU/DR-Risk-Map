str='0_0001'; % Wasserstein ambiguity set radius (theta)
rob_rad = 0.09; % Robot radius
safe_rad = 0.01; % Safety radius
K = 10; % Prediction Horizon
Ts = 0.1; % Sampling time
nx = 4; % Robot state dimension
ny = 2; % Robot output dimension 
nu = 2; % Control input dimension
Nobs = 4; % Number of static obstacles
L = 3; % Number of dynamic obstacles
obs_nx = 2; % Obstacle state dimension
obs_nu = 2; % Obstacle input dimension
obs_rad = 0.1; % Dynamic obstacle radius
Q = eye(ny); % Cost weight Q
R = 0.001*eye(nu); % Cost weight R
M_max = 10; % Maximum size of GP dataset
delta = 4e-4; % risk tolerance delta
x0 = [0.3; 0.2; 0; 0]; % Initial state
goal = [0.3 1.6 1.6 4.0 4.0;  
        0.2 0.2 4.0 4.0 5.0]; % List of waypoints
umin = [-4, -4]; umax = [4, 4];  % Control limits
xmin = [0, 0, -2, -2]; xmax = [10, 10, 2, 2]; % State limits
P = inv([0.7^2 0;0 0.8^2]); % Inverse of static obstacle radii matrix
save_video = true; % Turns on video recording
