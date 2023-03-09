function [bestCost,bestState] = steer(from,to)
    global L_r rob_rad Ts poscont ucost;
    expected_pose(1,:)=from.coord(1)+Ts*(poscont(1,:)*cos(from.coord(3)));
    expected_pose(2,:)=from.coord(2)+Ts*(poscont(1,:)*sin(from.coord(3)));
    expected_pose(3,:)=from.coord(3)+Ts*tan(poscont(2,:)).*poscont(1,:)/L_r;
    choose=(expected_pose(1,:)>0 & expected_pose(1,:)<13 & expected_pose(2,:)>20+rob_rad & expected_pose(2,:)<23.7-rob_rad) | (expected_pose(1,:)>20+rob_rad & expected_pose(1,:)<23.7-rob_rad & expected_pose(2,:)>0 & expected_pose(2,:)<13) | ...
                                   (expected_pose(1,:)>13 & expected_pose(1,:)<23.7-rob_rad & expected_pose(2,:)>13 & expected_pose(2,:)<23.7-rob_rad);
                               
    expected_pose1=expected_pose(:,choose);
    steerings=poscont(:,choose);
    
    distance=trajLength(expected_pose1,to);
    if ~isempty(from.used_controls)
        for i=1:size(from.used_controls,2)
            distance((abs(steerings-from.used_controls(1,i))<=10e-7) & (abs(steerings-from.used_controls(2,i))<=10e-7))=inf;
        end
    end
    
    [bestCost, bestIndex] = min(distance + ucost(:,choose));
    bestState = expected_pose1(:,bestIndex);
end