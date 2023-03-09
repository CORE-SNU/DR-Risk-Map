theta = 5e-3; % Wasserstein ambiguity set radius
alpha = 0.95; % CVaR confidence level
w_risk = 0; % 0 Risk weight
r_rrt = 1; % RRT* radius

startstate = [21.85 0 pi/2]; % Initial state
goalstate = [2 21.85 pi]; % Goal state
K = 10; % Maximum depth of the tree
Ts = 0.1; % Sampling time
time_to_grow = 3; % Allowable time to expand the tree
tryGoalProbability = 0.1; % probability for streering towards the goal instead of a random node
goal_threshold = 4; % Distance threshold for streering towards the goal

M_max = 10; % Maximum size of GP dataset
obs_nx = 2; % Obstacle state dimension
obs_nu = 2; % Obstacle input dimension
rob_rad = 1; % Robot radius
obs_rad = 1; % Obstacle radius
safe_rad = 0.1; % Safety radius
L = 1; % Number of Obstacles
r2=([rob_rad+obs_rad+safe_rad rob_rad+obs_rad+safe_rad]).^2; % Safety radius squared

delta=r2(1)*3/100; % 0 Risk tolerance

nx = 3; % Robto state dimension
L_r = 0.8; % Robot length
x_min = 0; x_max = 40; y_min = 0; y_max = 40; %Position limits
minAngle = -deg2rad(30); maxAngle = deg2rad(30); % Steering angle limits
minVel = 0; maxVel = 10; % Velocity limits
velIncrement = 0.1; % Velocity discretization step
angIncrement = 0.02; % Steering angle discretization step

poscont=combvec(minVel:velIncrement:maxVel,minAngle:angIncrement:maxAngle); % Discrete space of control inputs
ucost = 0.00001*sum(poscont.*poscont); % Control costs

save_video = true; % Turns on video recording