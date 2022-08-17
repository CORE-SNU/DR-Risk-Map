function q_near=near(nodes,q_rand)
   global w_risk K q_goal;
   ndist = [];
%    for j = 1:1:length(nodes)
%             smth(j)=w*max(nodes(j).risk);
%             smth1(j)=trajLength(nodes(j).coord,q_rand);
%             tmp = w*max(nodes(j).risk)+trajLength(nodes(j).coord,q_rand);
%             ndist = [ndist tmp];
%    end
   nc=[nodes.coord];
   nr=[nodes.risk];
   ncc=nc(:,[nodes.depth]<=K-1);
   ndist=w_risk*max(nr(:,[nodes.depth]<=K-1))+trajLength1(ncc,q_rand);
   if ~isempty(ndist)
       [val, idx] = min(ndist);
       q_near = nodes(all([nodes.coord]==ncc(:,idx)));
       index = nodes(all([nodes.coord]==ncc(:,idx))).index;
   else
       q_near = [];
       index=[];
   end
end