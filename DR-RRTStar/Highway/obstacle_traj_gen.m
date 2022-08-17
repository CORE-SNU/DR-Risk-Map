%  
clear all
close all
str1=sprintf(' \r\n')+"Ipopt 3.12.8: Converged to a locally infeasible point. Problem may be infeasible."+sprintf('\r\n');
% global L W minAngle angIncrement maxAngle minVel velIncrement maxVel Ts root new_pl vels angles poscont w q_goal K riskthreshold controller t xy21 xy21_var;
ii=[0];
nx=3;
startstate=[2 5.55];
startstate1=[2 5.55 0];
goalstate=[35 5.55];
goalstate1=[35 5.55 0];
%statobs = [centerx centery theta length width radius];
x_min=0;
x_max=80;
y_min=0;
y_max=11.1;
r=1;
Ts=0.1;
L=2; W=2;
K1=100;
K=10;
statobs=[-3 1.85 0 L W 0.85; 4 9.25 0 L W 0.85];
obs_state(:,:,1)=statobs(1,:)';
obs_state(:,:,2)=statobs(2,:)';
minAngle=-0.6; maxAngle=0.6; angIncrement=0.01; minVel=0; maxVel=10; velIncrement=0.05; minW=tan(minAngle)*maxVel/L; maxW=tan(maxAngle)*maxVel/L;
x_min=0; x_max=80; y_min=0; y_max=11.1; %limits

x=sdpvar(3,K);
u=sdpvar(2,K);
y=sdpvar(3,1);
obj=(x(1:3,K)-y)'*(x(1:3,K)-y);
const=[];
for k=1:K-1
    const=const+[x(1,k+1)==x(1,k)+Ts*u(1,k)*cos(x(3,k)),...
                 x(2,k+1)==x(2,k)+Ts*u(1,k)*sin(x(3,k)),...
                 x(3,k+1)==x(3,k)+Ts*u(2,k),...
                 minVel<=u(1,k)<=maxVel,...
                 minW<=u(2,k)<=maxW,...
                 -0.4<=x(3,k+1)-x(3,k)<=0.4];
    obj=obj+(x(1:3,k)-y)'*(x(1:3,k)-y)+0.1*u(:,k)'*u(:,k);
end

control=optimizer(const,obj,sdpsettings('solver','ipopt'),{x(:,1),y},{x,u});

figure
% set(gcf,'Position',[0,300,1035,345]);
% set(gca,'Position',[0.052434456928839,0.137956331862895,0.924157303370789,0.785723427417197]);
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

veh_traj1=plot(obs_state(1,1,1),obs_state(2,1,1),'Color',[150 150 150]/255,'LineStyle',':');
veh_traj2=plot(obs_state(1,1,2),obs_state(2,1,2),'Color',[150 150 150]/255,'LineStyle',':');
veh_traj1_pred=plot(obs_state(1,1,1),obs_state(2,1,1),'Color','r');
veh_traj2_pred=plot(obs_state(1,1,2),obs_state(2,1,2),'Color','r');

goal{1}=[30 35 55 60 90;1.85 5.55 5.55 9.25 9.25;0 0 0 0 0];
goal{2}=[17 22 40 47 90;9.25 5.55 5.55 1.85 1.85;0 0 0 0 0];
j1=1;
j2=1;
for k=1:K1
    if norm(obs_state(1:2,k,1)-goal{1}(1:2,j1))<=2 && j1<size(goal{1},2)
        j1=j1+1;
    end
    if norm(obs_state(1:2,k,2)-goal{2}(1:2,j2))<=2 && j2<size(goal{2},2)
        j2=j2+1;
    end
    sol{1}=control({obs_state(1:3,k,1),goal{1}(:,j1)});
    sol{2}=control({obs_state(1:3,k,2),goal{2}(:,j2)});
    obs_state(:,k+1,1)=[sol{1}{1}(:,2); statobs(1,4:end)'];
    obs_state(:,k+1,2)=[sol{2}{1}(:,2);  statobs(2,4:end)'];
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
    for kk=1:2
    Gp1{kk}(:,:,k)=[cos(obs_state(3,k,kk)) sin(obs_state(3,k,kk));cos(obs_state(3,k,kk)+pi) sin(obs_state(3,k,kk)+pi);cos(obs_state(3,k,kk)+pi/2) sin(obs_state(3,k,kk)+pi/2); cos(obs_state(3,k,kk)-pi/2) sin(obs_state(3,k,kk)-pi/2)];
    g_p1{kk}(1,k)=Gp1{kk}(1,:,k)*obs_state(1:2,k,kk)+statobs(kk,4)/2;
    g_p1{kk}(2,k)=Gp1{kk}(2,:,k)*obs_state(1:2,k,kk)+statobs(kk,4)/2;
    g_p1{kk}(3,k)=Gp1{kk}(3,:,k)*obs_state(1:2,k,kk)+statobs(kk,5)/2;
    g_p1{kk}(4,k)=Gp1{kk}(4,:,k)*obs_state(1:2,k,kk)+statobs(kk,5)/2;
    movobs11{kk}(1,:,k)=(inv(Gp1{kk}([1 3],:,k))*g_p1{kk}([1 3],k))';
    movobs11{kk}(2,:,k)=(inv(Gp1{kk}([1 4],:,k))*g_p1{kk}([1 4],k))';
    movobs11{kk}(3,:,k)=(inv(Gp1{kk}([2 4],:,k))*g_p1{kk}([2 4],k))';
    movobs11{kk}(4,:,k)=(inv(Gp1{kk}([2 3],:,k))*g_p1{kk}([2 3],k))';  
   if k~=1
       try
           set(movv1{kk},'xdata',movobs11{kk}(:,1,k));
           set(movv1{kk},'ydata',movobs11{kk}(:,2,k));
       catch
           movv1{kk}=fill(movobs11{kk}(:,1,k),movobs11{kk}(:,2,k),[0 .5 .5],'EdgeColor',[0 .5 .5],'FaceAlpha',0.05);
       end
   end
       try
           set(movv1{kk},'xdata',movobs11{kk}(:,1,k));
           set(movv1{kk},'ydata',movobs11{kk}(:,2,k));
       catch
           movv1{kk}=fill(movobs11{kk}(:,1,k),movobs11{kk}(:,2,k),[0 .5 .5],'EdgeColor',[0 .5 .5],'FaceAlpha',0.05);
       end
    end
   pause(0.1)
end
plot(obs_state(1,:,1),obs_state(2,:,1),'k');
plot(obs_state(1,:,2),obs_state(2,:,2),'k');