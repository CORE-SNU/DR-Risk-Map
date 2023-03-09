%% Initialize starting node of the tree to the robot's initial state
q_start.coord = startstate';
q_start.cost = 0;
q_start.parent = 0;
q_start.used_controls = [];
q_start.depth = 0;
q_start.risk = 0;
q_start.safe = (q_start.risk>delta);
q_start.children = [];
q_start.index = 1;
q_start.input = [];

%% Initialize the goal node of the tree
q_goal.coord = goalstate';

%% Initialize the root of the tree
nodes(1) = q_start;
root = q_start;

%% Initialize robot state
rob_state(:,1)=startstate';

%% Define a zero-mean GP prior with an RBF kernel
meanfunc = {@meanZero};
covfunc = {@covSEiso};
likfunc ={@likGauss};
hyp = struct('mean', [] , 'cov', [0 0], 'lik', -1);

%% Start recording 
if save_video==true
    writerObj = VideoWriter(['Simulations/theta_',num2str(theta),'.avi']);
    writerObj.FrameRate = 10;
    writerObj.Quality=100;
    open(writerObj)
end