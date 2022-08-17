q_start.coord = startstate';
q_start.cost = 0;
q_start.parent = 0;
q_start.open=1;
q_start.used_controls = [];
q_start.control=[];
q_start.depth=0;
q_start.risk=0;
q_start.isFree=(q_start.risk<=delta);
q_start.sons=[];
q_start.index=1;
q_start.input=[];
% 
% q_goal.coord = goalstate1';
% veh_coord=q_start.coord;
% nodes(1) = q_start;
% root = q_start;

q_goal.coord = goalstate';
nodes(1) = q_start;
veh_coord=zeros(nx,K1);
veh_coord(:,1)=startstate';


meanfunc = {@meanZero};
covfunc = {@covSEiso};
likfunc ={@likGauss};
hyp = struct('mean', [] , 'cov', [0 0], 'lik', -1);

writerObj = VideoWriter(['Simulations/theta1_',num2str(theta),'_new.avi']);
writerObj.FrameRate = 10;
writerObj.Quality=100;
open(writerObj)