clear all
close all

Set_Params
L = 2;
obs_state(:,:,1) = [0 3.08]';
obs_state(:,:,2) = [13 9]';

x=sdpvar(obs_nx,K);
u=sdpvar(obs_nx,K);
y=sdpvar(obs_nx,K);
sdpvar max_u;
obj=(x(:,K)-y(:,K))'*(x(:,K)-y(:,K));
const=[];
for k=1:K-1
    const=const+[x(1,k+1)==x(1,k)+Ts*u(1,k),...
                 x(2,k+1)==x(2,k)+Ts*u(2,k),...
                 -max_u <= u(1,k) <= max_u,...
                 -max_u <= u(2,k) <= max_u];
    obj=obj+(x(:,k)-y(:,k))'*(x(:,k)-y(:,k))+0.1*u(:,k)'*u(:,k);
end

control=optimizer(const,obj,sdpsettings('solver','fmincon'),{x(:,1),y,max_u},{x,u});

figure
set(gcf,'Color','white', 'Position',[-1406,77,893,869]);
set(gca,'Position',[0.043673012318029,0.052934407364787,0.912653975363942,0.928465592635213]);
xlabel('X')
ylabel('Y')
daspect([1 1 1])
axis([0 10 0 10])
hold on

for obs_idx=1:L
    obs_traj{obs_idx}=plot(obs_state(1,1,obs_idx),obs_state(2,1,obs_idx),'Color',[150 150 150]/255,'LineStyle',':');
    obs_traj_pred{obs_idx}=plot(obs_state(1,1,obs_idx),obs_state(2,1,obs_idx),'Color','r');
end

p1=0.08;p2=-0.16;p3=3.08;
obs_goal{1}=[0:0.15:15;p1*[0:0.15:15].^2+p2*[0:0.15:15]+p3];
obs_goal{2}=[13:-0.15:2;linspace(9,4,length(13:-0.15:2))];

plot(obs_goal{1}(1,:),obs_goal{1}(2,:),'k');
plot(obs_goal{2}(1,:),obs_goal{2}(2,:),'k');

for k=1:100
    if k+K-1>length(obs_goal{1})
        obs_goal{1}(:,k+K-1)=obs_goal{1}(:,k+K-2);
    end
    if k+K-1>length(obs_goal{2})
        obs_goal{2}(:,k+K-1)=obs_goal{2}(:,k+K-2);
    end

    for obs_idx=1:L
        [sol{obs_idx},diag1]=control({obs_state(1:2,k,obs_idx),obs_goal{1}(:,k:k+K-1),2});
        obs_state(:,k+1,obs_idx) = sol{obs_idx}{1}(:,2);
        obs_input(:,k,obs_idx)=sol{obs_idx}{2}(:,1);
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