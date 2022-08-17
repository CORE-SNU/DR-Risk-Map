timer=tic;
ii=0;
while toc(timer)<= time_to_grow
    i=i+1;
    ii=ii+1;
%Choose random sample
%The goal is chosen as the random sample with prob tryGoalProbability
% if rand()<=tryGoalProbability
%     q_rand=q_goal.coord;
% else
%     q_rand = [x_min+(x_max-x_min)*rand() y_min+(y_max-y_min)*rand() 2*pi*rand()]';
% end
%Find nearest node to grow from all available nodes
q_rand=q_rand_all{t}(:,ii);
if norm(root.coord(1:2)-q_goal.coord(1:2))<=4
    q_rand=q_goal.coord;
end

q_nearest=near(nodes_to_grow,q_rand);
%Extend the tree from q_near to the q_random
[bestCost,bestState,bestControl]=steer(q_nearest,q_rand);
if isempty(bestState)
    i=i-1;
    continue;
end
q_new.coord=bestState;
q_new.open=1;
q_new.depth=q_nearest.depth+1;
q_new.risk=ComputeRisk(q_new,q_new.depth);
q_new.isFree=(q_new.risk<=delta);
q_new.cost= q_nearest.cost+w*q_new.risk+trajLength(q_nearest.coord,q_new.coord);
q_new.control = bestControl;
if isnan(q_new.cost) || all(ismember(q_new.coord,[nodes.coord]))
    i=i-1;
    continue
end
q_near = [];
gc=[nodes_to_grow.coord];
nc=[q_new.coord];
q_near=nodes_to_grow(vecnorm(gc(1:2,:)- nc(1:2,:))<=r_rrt);
neighbor_count=length(q_near);
q_min = q_nearest;
C_min = q_new.cost;
newcontrol = q_new.control;
for k = 1:length(q_near)
    C_new=q_near(k).cost+w*q_new.risk+trajLength(q_near(k).coord,q_new.coord);
    [f1,newcont]=Feas(q_near(k).coord, q_new.coord);
    if C_new < C_min && f1
        q_min = q_near(k);
        C_min = C_new;
        newcontrol=newcont;
        hold on
    end
end
new_pl{i+1}=line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)],'Color', [150 150 150]/255, 'LineWidth', 1);

q_new.control=newcontrol;
q_new.cost=C_min;
q_new.parent = q_min.index;
q_new.index=i+1;
q_new.depth=nodes([nodes.index]==q_new.parent).depth+1;
[~,q_new.input]=Feas(q_min.coord, q_new.coord);
q_new.sons=[];
q_new.used_controls = [];
nodes([nodes.index]==q_new.parent).sons=[nodes([nodes.index]==q_new.parent).sons q_new.index];
nodes([nodes.index]==q_new.parent).used_controls=q_new.input;
nodes = [nodes q_new];
if q_new.isFree
   nodes_to_grow=[nodes_to_grow q_new];
   nodes_to_grow([nodes_to_grow.index]==q_new.parent).sons=[nodes_to_grow([nodes_to_grow.index]==q_new.parent).sons q_new.index];
   nodes_to_grow([nodes_to_grow.index]==q_new.parent).used_controls=q_new.input;
end
clear q_new q_nearest q_near;
end