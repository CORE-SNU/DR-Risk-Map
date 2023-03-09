% Plan a path from tree root to the goal

nc = [safe_nodes.coord];
distancetogoal = vecnorm(nc(1:2,:)-q_goal.coord(1:2));

intersection1 = lineSegmentIntersect([nc(1:2,:)' repmat(goalstate(1:2),size(nc,2),1)],[20+rob_rad/2 0 20+rob_rad/2 16.3]);
intersection2 = lineSegmentIntersect([nc(1:2,:)' repmat(goalstate(1:2),size(nc,2),1)],[0 20+rob_rad/2 16.3 20+rob_rad/2]);
intersects=intersection1.intAdjacencyMatrix | intersection2.intAdjacencyMatrix;
distancetogoal(intersects)=min(vecnorm(nc(1:2,intersects)-[20;16.3])+vecnorm([20;16.3]-[16.3;20])+vecnorm([16.3;20]-q_goal.coord(1:2)),min(vecnorm(nc(1:2,intersects)-[20;16.3])+vecnorm([20;16.3]-q_goal.coord(1:2)),vecnorm(nc(1:2,intersects)-[16.3;20])+vecnorm([16.3;20]-q_goal.coord(1:2))));
D = [safe_nodes.cost]+distancetogoal;
[val, idx] = min(D(2:end));
q_final = safe_nodes(idx+1);


BestTree=[];
j=1;

if size(safe_nodes,2)<=1
    Best=[safe_nodes(1) safe_nodes(1)];
else
    while q_final.index ~= root.index
        start = q_final.parent;
        best_pl(j)=line([q_final.coord(1), safe_nodes([safe_nodes.index]==start).coord(1)], [q_final.coord(2), safe_nodes([safe_nodes.index]==start).coord(2)], 'Color', 'r', 'LineStyle','--',  'LineWidth', 2);
        hold on
        BestTree=[BestTree safe_nodes([safe_nodes.index]==start)];
        BestTree=[BestTree q_final];
        q_final = safe_nodes([safe_nodes.index]==start);
        j=j+1;
    end
    BestTree=[BestTree safe_nodes(1)];
    Best=flip(BestTree);
end