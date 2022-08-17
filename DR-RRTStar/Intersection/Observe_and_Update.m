if t~=1
%     if size(nodes_to_grow,2)>1
    old_root=root;
    root=Best(2);
    root.cost = 0;
    root.parent = 0;
    root.open=1;
    root.risk=0;
    root.isFree=(q_start.risk<=delta);
    root.input=[];
    root.depth=0;
    if root.index~=old_root.index
        nodes=delete_unreachables(nodes,root,old_root);
        newdepth=num2cell([nodes.depth]-1);
        [nodes.depth]=newdepth{:};
    end
%     end
else
    q_start.coord = veh_coord(:,t);
    q_start.cost = 0;
    q_start.parent = 0;
    q_start.open=1;
    q_start.used_controls = [];
    q_start.depth=0;
    q_start.risk=0;
    q_start.isFree=(q_start.risk<=delta);
    q_start.sons=[];
    q_start.index=1;
    q_start.input=[];
    nodes(t) = q_start;
    root = q_start;
end

function nodes=delete_unreachables(nodes,newroot,old_root)
global new_pl
node_stack=old_root.index;
while ~isempty(node_stack)
    index=([nodes.index]==node_stack(end));
    current_node=nodes(index);
    node_stack(end)=[];
    if ~isempty(current_node.sons) && current_node.index~=newroot.index
                node_stack=[node_stack current_node.sons];
    end
    if current_node.index~=newroot.index 
        nodes(index)=[];
        delete(new_pl{current_node.index});
    end
end
delete(new_pl{newroot.index});
end

