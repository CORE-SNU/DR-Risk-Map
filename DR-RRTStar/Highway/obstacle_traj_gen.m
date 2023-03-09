clear all
close all

Set_Params

obs_state(:,:,1) = [-3 1.85 0]';
obs_state(:,:,2) = [4 9.25 0]';

minW = tan(-0.6)*maxVel/2; maxW = tan(0.6)*maxVel/2;

x = sdpvar(obs_nx,K);
u = sdpvar(obs_nu,K);
y = sdpvar(obs_nx,1);

obj = (x(1:3,K)-y)'*(x(1:3,K)-y);
const = [];
for k=1:K-1
    const=const+[x(1,k+1) == x(1,k)+Ts*u(1,k)*cos(x(3,k)),...
                 x(2,k+1) == x(2,k)+Ts*u(1,k)*sin(x(3,k)),...
                 x(3,k+1) == x(3,k)+Ts*u(2,k),...
                 minVel <= u(1,k) <= maxVel,...
                 minW <= u(2,k) <= maxW,...
                 -0.4 <= x(3,k+1)-x(3,k) <= 0.4];
    obj = obj + (x(1:3,k)-y)'*(x(1:3,k)-y)+0.1*u(:,k)'*u(:,k);
end

control=optimizer(const,obj,sdpsettings('solver','fmincon'),{x(:,1),y},{x,u});

figure

set(gcf,'Position',[766,42,187,954]);
set(gcf,'Color','white');
axis([x_min x_max y_min y_max]);
hold on
xlabel('X')
ylabel('Y')
set(gca,'CameraUpVector',[1 0 0],'YDir','reverse','Position',[0.072724312001303,0.020336772917979,0.790823970037456,0.975041830035187]);
set(gca,'Color',[0.9 0.9 0.9]);
lane1=line([0 x_max],[0 0],'LineWidth',2,'Color','k');
lane2=line([0 x_max],[3.7 3.7],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane3=line([0 x_max],[7.4 7.4],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane4=line([0 x_max],[y_max y_max],'LineWidth',2,'Color','k');


for obs_idx=1:L
    obs_traj{obs_idx} = plot(obs_state(1,1,obs_idx),obs_state(2,1,obs_idx),'Color',[150 150 150]/255,'LineStyle',':');
    obs_traj_pred{obs_idx} = plot(obs_state(1,1,obs_idx),obs_state(2,1,obs_idx),'Color','r');
end

goal{1}=[30 35 55 60 90;1.85 5.55 5.55 9.25 9.25;0 0 0 0 0];
goal{2}=[17 22 40 47 90;9.25 5.55 5.55 1.85 1.85;0 0 0 0 0];
j = [1 1];
for k=1:100
    if norm(obs_state(1:2,k,1)-goal{1}(1:2,j(1)))<=2 && j(1)<size(goal{1},2)
        j(1)=j(1)+1;
    end
    if norm(obs_state(1:2,k,2)-goal{2}(1:2,j(2)))<=2 && j(2)<size(goal{2},2)
        j(2)=j(2)+1;
    end

    for obs_idx=1:L
        sol{obs_idx}=control({obs_state(1:3,k,obs_idx),goal{obs_idx}(:,j(obs_idx))});
        obs_state(:,k+1,obs_idx) = sol{obs_idx}{1}(:,2);
        obs_input(:,k,obs_idx) = sol{obs_idx}{2}(:,1);
        set(obs_traj{obs_idx}, 'xdata', obs_state(1,1:k+1,obs_idx), 'ydata', obs_state(2,1:k+1,obs_idx));
        set(obs_traj_pred{obs_idx}, 'xdata', sol{obs_idx}{1}(1,:), 'ydata', sol{obs_idx}{1}(2,:));  
        if k~=1
            set(obs_obj{obs_idx}, 'Position',[obs_state(1,k+1,obs_idx)-obs_rad obs_state(2,k+1,obs_idx)-obs_rad 2*obs_rad 2*obs_rad]);
        else
            obs_obj{obs_idx} = rectangle('Position',[obs_state(1,1,obs_idx)-obs_rad obs_state(2,1,obs_idx)-obs_rad 2*obs_rad 2*obs_rad],'FaceColor',[0 .5 .5 0.05],'EdgeColor',[0 .5 .5],'Curvature',[1 1]);
        end
    end
    pause(0.1)
end

for obs_idx=1:L
    plot(obs_state(1,:,1),obs_state(2,:,1),'k');
end

save('obs_traj.mat', 'obs_input', 'obs_state')