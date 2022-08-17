nc=[nodes_to_grow.coord];
D=[nodes_to_grow.cost]+vecnorm(nc(1:3,:)-q_goal.coord(1:3));
% try
%     nd=[nodes_to_grow.depth];
%     [val,idx]=min(D(nd==10));
%     nc1=nodes_to_grow(nd==10);
%     q_final=nc1(idx);
% catch
    [val, idx] = min(D(2:end));
    q_final = nodes_to_grow(idx+1);
% end
BestTree=[];
j=1;
if size(nodes_to_grow,2)<=1
    Best=[nodes_to_grow(1) nodes_to_grow(1)];
else
while q_final.index ~= root.index
    start = q_final.parent;
    best_pl(j)=line([q_final.coord(1), nodes_to_grow([nodes_to_grow.index]==start).coord(1)], [q_final.coord(2), nodes_to_grow([nodes_to_grow.index]==start).coord(2)], 'Color', 'r', 'LineStyle','--',  'LineWidth', 2);
    hold on
    BestTree=[BestTree nodes_to_grow([nodes_to_grow.index]==start)];
    BestTree=[BestTree q_final];
    q_final = nodes_to_grow([nodes_to_grow.index]==start);
    j=j+1;
end
BestTree=[BestTree nodes_to_grow(1)];
Best=flip(BestTree);
end
% k=size(BestTree,2);
% for j=1:k
%     Best(j)=BestTree(k-j+1);
% end