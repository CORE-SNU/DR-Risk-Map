nc=[nodes_to_grow.coord];
distancetogoal=vecnorm(nc(1:2,:)-q_goal.coord(1:2));
intersection1=lineSegmentIntersect([nc(1:2,:)' repmat(goalstate(1:2),size(nc,2),1)],[20+W/2 0 20+W/2 16.3]);
intersection2=lineSegmentIntersect([nc(1:2,:)' repmat(goalstate(1:2),size(nc,2),1)],[0 20+W/2 16.3 20+W/2]);
intersects=intersection1.intAdjacencyMatrix | intersection2.intAdjacencyMatrix;
distancetogoal(intersects)=min(vecnorm(nc(1:2,intersects)-[20;16.3])+vecnorm([20;16.3]-[16.3;20])+vecnorm([16.3;20]-q_goal.coord(1:2)),min(vecnorm(nc(1:2,intersects)-[20;16.3])+vecnorm([20;16.3]-q_goal.coord(1:2)),vecnorm(nc(1:2,intersects)-[16.3;20])+vecnorm([16.3;20]-q_goal.coord(1:2))));
D=[nodes_to_grow.cost]+distancetogoal;
% D=trajLength1(nc,q_goal.coord);

% try
%     nd={nodes_to_grow.sons};
%     maxd=1-cellfun(@isempty,{nodes_to_grow.sons});
%     nd1=[nodes_to_grow.depth];
%     maxd1=max(nd1);
%     [val,idx]=min(D((nd1==maxd1) | maxd));
%     nc1=nodes_to_grow((nd1==maxd1) | maxd);
%     q_final=nc1(idx);
% catch
    [val, idx] = min(D(2:end));
    q_final = nodes_to_grow(idx+1);
% end

% [val, idx] = min(D(2:end));
BestTree=[];
% q_final = nodes_to_grow(idx+1);
j=1;
% if size(nodes_to_grow,2)<=1
%     Best=[nodes_to_grow(1) nodes_to_grow(1)];
% else
while q_final.index ~= root.index
    start = q_final.parent;
    best_pl(j)=line([q_final.coord(1), nodes_to_grow([nodes_to_grow.index]==start).coord(1)], [q_final.coord(2), nodes_to_grow([nodes_to_grow.index]==start).coord(2)], 'Color', 'r', 'LineStyle','--',  'LineWidth', 2);
    hold on
%     BestTree=[BestTree nodes_to_grow([nodes_to_grow.index]==start)];
    BestTree=[BestTree q_final];
    q_final = nodes_to_grow([nodes_to_grow.index]==start);
    j=j+1;
end
BestTree=[BestTree nodes_to_grow(1)];
Best=flip(BestTree);
% end
% k=size(BestTree,2);
% for j=1:k
%     Best(j)=BestTree(k-j+1);
% end