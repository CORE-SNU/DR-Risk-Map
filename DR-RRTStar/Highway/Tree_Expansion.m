timer=tic;
ii=0;
while toc(timer)<= time_to_grow
    i=i+1;
    ii=ii+1;
    q_rand=q_rand_all{t}(:,ii); % Choose a random sample
    
    % If the robot is close to the goal, steer to the goal
    if norm(root.coord(1:2)-q_goal.coord(1:2)) <= goal_threshold
        q_rand = q_goal.coord;
    end
    
    % Find nearest node to grow from all available nodes
    q_nearest=near(safe_nodes,q_rand);

    % Extend the tree from q_nearest to the q_random
    [bestCost,bestState]=steer(q_nearest,q_rand);

    % Update the depth, risk and cost for the new node
    q_new.coord = bestState;
    q_new.depth = q_nearest.depth+1;
    q_new.risk = ComputeRisk(q_new,q_new.depth);
    q_new.safe = (q_new.risk<=delta);
    q_new.cost = q_nearest.cost+w_risk*q_new.risk+trajLength(q_nearest.coord,q_new.coord);

    % Rollback if the new node already exists
    if isnan(q_new.cost) || all(ismember(q_new.coord,[nodes.coord]))
        i=i-1;
        continue
    end

    % Find all nodes from the safe subtree within r_rrt distance from the
    % new node
    q_near = [];
    gc = [safe_nodes.coord];
    nc = [q_new.coord];
    q_near = safe_nodes(vecnorm(gc(1:2,:) - nc(1:2,:))<=r_rrt);
    neighbor_count = length(q_near);

    % Compute the cost to the new node via neiboring nodes
    q_min = q_nearest;
    C_min = q_new.cost;
    for k = 1:length(q_near)
        C_new=q_near(k).cost+w_risk*q_new.risk+trajLength(q_near(k).coord,q_new.coord);
        if C_new < C_min && Feas(q_near(k).coord, q_new.coord)
            q_min = q_near(k);
            C_min = C_new;
            hold on
        end
    end

    % Render the node with the smallest cost to the new node
    new_pl{i+1} = line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)],'Color', [150 150 150]/255, 'LineWidth', 1);
    
    % Update the parent of q_new to q_min and the corresponding cost
    q_new.cost = C_min;
    q_new.parent = q_min.index;
    q_new.index = i+1;
    q_new.depth = nodes([nodes.index]==q_new.parent).depth+1;
    [~,q_new.input] = Feas(q_min.coord, q_new.coord); % Find a feasible control input to steer from q_min to q_new
    q_new.children = [];
    q_new.used_controls = [];

    % Add q_new as a children node to its parent
    nodes([nodes.index]==q_new.parent).children = [nodes([nodes.index]==q_new.parent).children q_new.index];

    % Update the list of control inputs used by the parent node
    nodes([nodes.index]==q_new.parent).used_controls = q_new.input;

    % Update the tree
    nodes = [nodes q_new];

    % If the new node is safe, add it to the safe subtree
    if q_new.safe
       safe_nodes=[safe_nodes q_new];
    end

    % TODO: Implement the rewiring (skipped due to time complexity)
    
    clear q_new q_nearest q_near;
end