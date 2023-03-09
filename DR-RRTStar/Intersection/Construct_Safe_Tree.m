% Find all nodes with depth less than the horizon K
depthok = find([nodes.depth]<=K);

% For each node compute DR-CVaR and determine whether it is safe
for j = depthok
    nodes(j).risk=ComputeRisk(nodes(j),nodes(j).depth);
    nodes(j).safe=(nodes(j).risk<=delta);
end

% Generate safe subtree
safe_nodes=[];
node_stack=root.index;
while ~isempty(node_stack)
    index = ([nodes.index]==node_stack(end));
    current_node = nodes(index);
    node_stack(end)=[];
    if ~isempty(current_node.children) && current_node.depth<=K && current_node.safe
        node_stack=[node_stack current_node.children];
        if nodes(index).depth~=0
            nodes(index).cost=nodes([nodes.index]==nodes(index).parent).cost+w_risk*nodes(index).risk+trajLength(nodes([nodes.index]==nodes(index).parent).coord,nodes(index).coord);
        end
        safe_nodes=[safe_nodes nodes(index)];
        if current_node.index~=root.index
            set(new_pl{current_node.index},'Color','b')
        end
    elseif current_node.index==root.index
        safe_nodes=[safe_nodes nodes([nodes.index]==root.index)];        
    end
end