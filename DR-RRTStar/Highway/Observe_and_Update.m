if t~=1
    old_root = root; % Saves the current root

    % Sets the new root to the current robot state
    root = Best(2);
    root.cost = 0; 
    root.parent = 0;
    root.risk = 0;
    root.safe = (q_start.risk<=delta);
    root.input = [];
    root.depth = 0;

    % If the root has changed, remove unreachable nodes and update depths for all nodes in the tree
    if root.index~=old_root.index
        nodes = delete_unreachables(nodes,root,old_root);
        newdepth = num2cell([nodes.depth]-1);
        [nodes.depth] = newdepth{:};
    end
end

function nodes = delete_unreachables(nodes,newroot,old_root)
    global new_pl
    node_stack=old_root.index;
    while ~isempty(node_stack)
        index=([nodes.index]==node_stack(end));
        current_node=nodes(index);
        node_stack(end)=[];
        if ~isempty(current_node.children) && current_node.index~=newroot.index
                    node_stack=[node_stack current_node.children];
        end
        if current_node.index~=newroot.index 
            nodes(index)=[];
            delete(new_pl{current_node.index});
        end
    end
    delete(new_pl{newroot.index});
end

