function [bestCost,bestState] = steer(from,to)
    global L_r Ts poscont ucost;
    expected_pose(1,:)=from.coord(1)+Ts*(poscont(1,:)*cos(from.coord(3)));
    expected_pose(2,:)=from.coord(2)+Ts*(poscont(1,:)*sin(from.coord(3)));
    expected_pose(3,:)=from.coord(3)+Ts*tan(poscont(2,:)).*poscont(1,:)/L_r;

    distance=trajLength(expected_pose,to);
    if ~isempty(from.used_controls)
        for i=1:size(from.used_controls,2)
            distance((abs(poscont(1,:)-from.used_controls(1,i))<=10e-7) & (abs(poscont(2,:)-from.used_controls(2,i))<=10e-7))=inf;
        end
    end

    [bestCost, bestIndex] = min(distance + ucost);
    bestState = expected_pose(:,bestIndex);
end