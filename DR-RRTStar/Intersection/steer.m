function [bestCost,bestState,bestControl] = steer(from,to)
global L W Ts poscont;
% tic
bestState=[];
bestCost=inf;    
expected_pose(1,:)=from.coord(1)+Ts*(poscont(1,:)*cos(from.coord(3)));
expected_pose(2,:)=from.coord(2)+Ts*(poscont(1,:)*sin(from.coord(3)));
expected_pose(3,:)=from.coord(3)+Ts*poscont(2,:);
%extract only feasibles
% expected_pose1=expected_pose(:,(expected_pose(1,:)>20+W/2 & expected_pose(1,:)<23.7-W/2 & expected_pose(2,:)>0 & expected_pose(2,:)<16.3+W/2) | (expected_pose(2,:)>20+W/2 & expected_pose(2,:)<23.7-W/2 & expected_pose(1,:)>0 & expected_pose(1,:)<16.3+W/2) | (expected_pose(2,:)>16.3+W/2 & expected_pose(2,:)<23.7 & expected_pose(1,:)>16.3+W/2 & expected_pose(1,:)<23.7)); 
choose=(expected_pose(1,:)>0 & expected_pose(1,:)<13 & expected_pose(2,:)>20+W/2 & expected_pose(2,:)<23.7-W/2) | (expected_pose(1,:)>20+W/2 & expected_pose(1,:)<23.7-W/2 & expected_pose(2,:)>0 & expected_pose(2,:)<13) | ...
                               (expected_pose(1,:)>13 & expected_pose(1,:)<23.7-W/2 & expected_pose(2,:)>13 & expected_pose(2,:)<23.7-W/2);
                           %& vecnorm(expected_pose(1:2,:)-[13;13])>3.3+W/2+3.7 & vecnorm(expected_pose(1:2,:)-[27;13])>3.3+W/2);
                           
expected_pose1=expected_pose(:,choose);
steerings=poscont(:,choose);
% expected_pose1=expected_pose(:,(expected_pose(1,:)>20+W/2 & expected_pose(1,:)<23.7-W/2 & expected_pose(2,:)>0 & expected_pose(2,:)<16.3+W/2) | (expected_pose(2,:)>20+W/2 & expected_pose(2,:)<23.7-W/2 & expected_pose(1,:)>0 & expected_pose(1,:)<16.3+W/2) | (expected_pose(2,:)>16.3+W/2 & expected_pose(2,:)<23.7 & expected_pose(1,:)>16.3+W/2 & expected_pose(1,:)<23.7)); 
distance=trajLength1(expected_pose1,to);
% if ~isempty(from.used_controls)
%     for i=1:size(from.used_controls,2)
%         distance((abs(poscont(1,:)-from.used_controls(1,i))<=10e-7) & (abs(poscont(2,:)-from.used_controls(2,i))<=10e-7))=inf;
%     end
% end
[bestCost, bestIndex] = min(distance);
% % % % R=[cos(expected_pose1(3,bestIndex)) -sin(expected_pose1(3,bestIndex));sin(expected_pose1(3,bestIndex)) cos(expected_pose1(3,bestIndex))];
% % % % movobs_veh1=(expected_pose1(1:2,bestIndex)+R*[L/2 L/2 -L/2 -L/2;W/2 -W/2 -W/2 W/2])';
% % % % if all(movobs_veh1(:,1)>19 & movobs_veh1(:,1)<23.7) || all(movobs_veh1(:,2)>19 & movobs_veh1(:,2)<23.7) || all(movobs_veh1(:,2)<23.7 & movobs_veh1(:,2)>16.3 & movobs_veh1(:,1)>16.3 & movobs_veh1(:,1)<23.7)
        bestState = expected_pose1(:,bestIndex);
        bestControl = steerings(:,bestIndex);
% % % % end
% toc
% % % sol=steercontrol({from.coord,to});
% % % bestState=sol{1};
% % % bestCost=sol{2};
end