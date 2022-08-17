%  
clear all
close all
x_min=0;
x_max=10;
y_min=0;
y_max=10;
r=1;
Ts=0.1;
W=0.25;
K1=100;
K=10;
statobs=[0 3.08 W;13 9 W];
p1=0.08;p2=-0.16;p3=3.08;
goal{1}=[0:0.15:15;p1*[0:0.15:15].^2+p2*[0:0.15:15]+p3];
goal{2}=[13:-0.15:2;linspace(9,4,length(13:-0.15:2))];

obs_state(:,:,1)=statobs(1,:)';
obs_state(:,:,2)=statobs(2,:)';

x=sdpvar(2,K);
u=sdpvar(2,K);
y=sdpvar(2,K);
sdpvar maxVel;
obj=(x(:,K)-y(:,K))'*(x(:,K)-y(:,K));
const=[];
for k=1:K-1
    const=const+[x(1,k+1)==x(1,k)+Ts*u(1,k),...
                 x(2,k+1)==x(2,k)+Ts*u(2,k),...
                 -maxVel<=u(1,k)<=maxVel,...
                 -maxVel<=u(2,k)<=maxVel];
    obj=obj+(x(:,k)-y(:,k))'*(x(:,k)-y(:,k))+0.1*u(:,k)'*u(:,k);
end

control=optimizer(const,obj,sdpsettings('solver','ipopt'),{x(:,1),y,maxVel},{x,u});

figure
set(gcf,'Color','white', 'Position',[-1406,77,893,869]);
set(gca,'Position',[0.043673012318029,0.052934407364787,0.912653975363942,0.928465592635213]);
xlabel('X')
ylabel('Y')
daspect([1 1 1])
axis([0 10 0 10])
hold on

veh_traj1=plot(obs_state(1,1,1),obs_state(2,1,1),'Color',[150 150 150]/255,'LineStyle',':');
veh_traj2=plot(obs_state(1,1,2),obs_state(2,1,2),'Color',[150 150 150]/255,'LineStyle',':');
veh_traj1_pred=plot(obs_state(1,1,1),obs_state(2,1,1),'Color','r');
veh_traj2_pred=plot(obs_state(1,1,2),obs_state(2,1,2),'Color','r');


plot(goal{1}(1,:),goal{1}(2,:),'k');
plot(goal{2}(1,:),goal{2}(2,:),'k');

j1=1;
j2=1;
for k=1:K1
    if k+K-1>length(goal{1})
        goal{1}(:,k+K-1)=goal{1}(:,k+K-2);
    end
    if k+K-1>length(goal{2})
        goal{2}(:,k+K-1)=goal{2}(:,k+K-2);
    end
    [sol{1},diag1]=control({obs_state(1:2,k,1),goal{1}(:,k:k+K-1),2});
    [sol{2},diag2]=control({obs_state(1:2,k,2),goal{2}(:,k:k+K-1),2});
    obs_state(:,k+1,1)=[sol{1}{1}(:,2); statobs(1,3:end)'];
    obs_state(:,k+1,2)=[sol{2}{1}(:,2);  statobs(2,3:end)'];
    obs_input(:,k,1)=sol{1}{2}(:,1);
    obs_input(:,k,2)=sol{2}{2}(:,1);
    set(veh_traj1, 'xdata', obs_state(1,1:k+1,1));
    set(veh_traj1, 'ydata', obs_state(2,1:k+1,1));
    set(veh_traj2, 'xdata', obs_state(1,1:k+1,2));
    set(veh_traj2, 'ydata', obs_state(2,1:k+1,2));
    set(veh_traj1_pred, 'xdata', sol{1}{1}(1,:));
    set(veh_traj1_pred, 'ydata', sol{1}{1}(2,:));
    set(veh_traj2_pred, 'xdata', sol{2}{1}(1,:));
    set(veh_traj2_pred, 'ydata', sol{2}{1}(2,:));
    for kk=1:size(statobs,1)
   if k~=1
       try
            set(movv1{kk},'Position',[obs_state(1,k,kk)-W/2 obs_state(2,k,kk)-W/2 W W]);
       catch
           movv1{kk}=rectangle('Position',[obs_state(1,k,kk)-W/2 obs_state(2,k,kk)-W/2 W W],'FaceColor',[0 .5 .5 0.05],'EdgeColor',[0 .5 .5],'Curvature',[1 1]);
       end
   end
       try
           set(movv1{kk},'Position',[obs_state(1,k,kk)-W/2 obs_state(2,k,kk)-W/2 W W]);
       catch
           movv1{kk}=rectangle('Position',[obs_state(1,k,kk)-W/2 obs_state(2,k,kk)-W/2 W W],'FaceColor',[0 .5 .5 0.05],'EdgeColor',[0 .5 .5],'Curvature',[1 1]);
       end
    end
   pause(0.1)
end
plot(obs_state(1,:,1),obs_state(2,:,1),'k');
plot(obs_state(1,:,2),obs_state(2,:,2),'k');