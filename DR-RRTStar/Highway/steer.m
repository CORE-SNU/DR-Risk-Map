function [bestCost,bestState] = steer(from,to)
global L W Ts poscont ucost;
% tic
bestState=[];
bestCost=inf;    
expected_pose(1,:)=from.coord(1)+Ts*(poscont(1,:)*cos(from.coord(3)));
expected_pose(2,:)=from.coord(2)+Ts*(poscont(1,:)*sin(from.coord(3)));
expected_pose(3,:)=from.coord(3)+0.8*Ts*poscont(2,:);
distance=trajLength1(expected_pose,to);
if ~isempty(from.used_controls)
    for i=1:size(from.used_controls,2)
        distance((abs(poscont(1,:)-from.used_controls(1,i))<=10e-7) & (abs(poscont(2,:)-from.used_controls(2,i))<=10e-7))=inf;
    end
end
[bestCost, bestIndex] = min(distance + ucost);
bestState = expected_pose(:,bestIndex);
% toc
% % % sol=steercontrol({from.coord,to});
% % % bestState=sol{1};
% % % bestCost=sol{2};
end