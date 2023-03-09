function q_near=near(nodes,q_rand)
   global w_risk K;
   ndist = [];
   nc=[nodes.coord];
   nr=[nodes.risk];
   ncc=nc(:,[nodes.depth]<=K-1);
   ndist=w_risk*max(nr(:,[nodes.depth]<=K-1))+trajLength(ncc,q_rand);
   if ~isempty(ndist)
       [val, idx] = min(ndist);
       q_near = nodes(all([nodes.coord]==ncc(:,idx)));
       index = nodes(all([nodes.coord]==ncc(:,idx))).index;
   else
       q_near = [];
       index=[];
   end
end