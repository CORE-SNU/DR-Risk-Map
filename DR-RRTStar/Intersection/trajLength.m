function distance_from_goal=trajLength(pose,goal)
global root;
pose_euclide_distance=sqrt((pose(1:2)-goal(1:2))'*(pose(1:2)-goal(1:2)));
root_euclide_distance=sqrt((root.coord(1:2)-goal(1:2))'*(root.coord(1:2)-goal(1:2)));
position_improvement = pose_euclide_distance/root_euclide_distance;
rotation_diff = atan2(goal(2)-pose(2),goal(1)-pose(1))-pose(3);
rotation_diff(rotation_diff > pi) = rotation_diff - 2*pi;
rotation_diff(rotation_diff < -pi) = rotation_diff + 2*pi;
distance_from_goal = position_improvement+0.3*abs(rotation_diff);
end
