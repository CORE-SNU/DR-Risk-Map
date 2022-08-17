depthok=find([nodes.depth]<=K);
for j=depthok
    nodes(j).risk=ComputeRisk(nodes(j),nodes(j).depth);
    nodes(j).isFree=(nodes(j).risk<=delta);
end

nodes_to_grow=[];
node_stack=root.index;
while ~isempty(node_stack)
    index=([nodes.index]==node_stack(end));
    current_node=nodes(index);
    node_stack(end)=[];
    if ~isempty(current_node.sons) && current_node.depth<=K && current_node.isFree
        node_stack=[node_stack current_node.sons];
        if nodes(index).depth~=0
            nodes(index).cost=nodes([nodes.index]==nodes(index).parent).cost+w_risk*nodes(index).risk+trajLength(nodes([nodes.index]==nodes(index).parent).coord,nodes(index).coord);
        end
        nodes_to_grow=[nodes_to_grow nodes(index)];
        if current_node.index~=root.index
            set(new_pl{current_node.index},'Color','b')
        end
    elseif current_node.index==root.index
        nodes_to_grow=[nodes_to_grow nodes([nodes.index]==root.index)];        
    end
end