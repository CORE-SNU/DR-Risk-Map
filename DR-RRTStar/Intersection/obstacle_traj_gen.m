%  
clear all
close all
str1=sprintf(' \r\n')+"Ipopt 3.12.8: Converged to a locally infeasible point. Problem may be infeasible."+sprintf('\r\n');
% global L W minAngle angIncrement maxAngle minVel velIncrement maxVel Ts root new_pl vels angles poscont w q_goal K riskthreshold controller t xy21 xy21_var;
ii=[0];
x_min=0;
x_max=40;
y_min=0;
y_max=40;
r=1;
Ts=0.1;
W=2;
K1=100;
K=10;
statobs=[5 18.15 W;42 21.85 W];
obs_state(:,:,1)=statobs(1,:)';
obs_state(:,:,2)=statobs(2,:)';
% minAngle=-0.6; maxAngle=0.6; angIncrement=0.01; minVel=0; maxVel=10; velIncrement=0.05; minW=tan(minAngle)*maxVel/W; maxW=tan(maxAngle)*maxVel/W;

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
set(gcf,'Color','white', 'Position',[197.6666666666667,67,664.6666666666666,540.6666666666666]);
set(gca,'Position',[0.073588709677419,0.083333333333333,0.884072580645161,0.884328358208955]);
axis([x_min x_max y_min y_max]);
hold on
xlabel('X')
ylabel('Y')
rectangle('Position',[0 0 40 40], 'FaceColor',[0.9 0.9 0.9],'EdgeColor','none')
rectangle('Position',[9.7 23.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[23.7 23.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[23.7 23.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[9.7 9.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[23.7 9.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)

rectangle('Position',[0 0 13 16.3],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[13 0 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[23.7 0 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[27 0 13 16.3],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[0 23.7 13 16.3],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[13 27 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[23.7 27 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[27 23.7 13 16.3],'FaceColor','w','EdgeColor','none') 

lane1=line([0 13],[16.3 16.3],'LineWidth',2,'Color','k');
lane2=line([0 13],[20 20],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane3=line([0 13],[23.7 23.7],'LineWidth',2,'Color','k');
lane4=line([27 40],[16.3 16.3],'LineWidth',2,'Color','k');
lane5=line([27 40],[20 20],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane6=line([27 40],[23.7 23.7],'LineWidth',2,'Color','k');
lane7=line([16.3 16.3],[0 13],'LineWidth',2,'Color','k');
lane8=line([20 20],[0 13],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane9=line([23.7 23.7],[0 13],'LineWidth',2,'Color','k');
lane10=line([16.3 16.3],[27 40],'LineWidth',2,'Color','k');
lane11=line([20 20],[27 40],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane12=line([23.7 23.7],[27 40],'LineWidth',2,'Color','k');

veh_traj1=plot(obs_state(1,1,1),obs_state(2,1,1),'Color',[150 150 150]/255,'LineStyle',':');
veh_traj2=plot(obs_state(1,1,2),obs_state(2,1,2),'Color',[150 150 150]/255,'LineStyle',':');
veh_traj1_pred=plot(obs_state(1,1,1),obs_state(2,1,1),'Color','r');
veh_traj2_pred=plot(obs_state(1,1,2),obs_state(2,1,2),'Color','r');


goal{2}=[42:-0.7:26 4.15*cos(-pi/2:-0.1:-pi)+26 repmat(21.85,1,length(26:0.7:80));repmat(21.85,1,length(42:-0.7:26)) 4.15*sin(-pi/2:-0.1:-pi)+26 26:0.7:80];
goal{1}=[5:0.7:16 2.15*cos(pi/2:-0.2:0)+16.5 repmat(18.65,1,length(15:-0.7:0)); repmat(18.15,1,length(5:0.7:16)) 2.15*sin(pi/2:-0.2:0)+16 15:-0.7:0];
plot(goal{1}(1,:),goal{1}(2,:),'k');
j1=1;
j2=1;
for k=1:K1
    if norm(obs_state(1:2,k,1)-[21.85;42])<=0.5
        break
    end
    if abs(obs_state(1,k,1)-17)<=0.5
        goal{1}(:,k:k+length([4.85*cos(-pi/2:0.1:0)+17 repmat(21.85,1,length(23:0.7:42))])-1)=[4.85*cos(-pi/2:0.1:0)+17 repmat(21.85,1,length(23:0.7:42)); 4.85*sin(-pi/2:0.1:0)+23 23:0.7:42];
        goal{1}(:,length([4.85*cos(-pi/2:0.1:0)+17 repmat(21.85,1,length(23:0.7:42))])+1:length([4.85*cos(-pi/2:0.1:0)+17 repmat(21.85,1,length(23:0.7:42))])+50)=repmat([21.85;42],1,50);
        plot(goal{1}(1,:),goal{1}(2,:),'k');
    end
%     if norm(obs_state(1:2,k,2)-goal{2}(1:2,j2))<=2 && j2<size(goal{2},2)
%         j2=j2+1;
%     end
    sol{1}=control({obs_state(1:2,k,1),goal{1}(:,k:k+K-1),10});
    sol{2}=control({obs_state(1:2,k,2),goal{2}(:,k:k+K-1),10});
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
%     Gp1{kk}(:,:,k)=[cos(obs_state(3,k,kk)) sin(obs_state(3,k,kk));cos(obs_state(3,k,kk)+pi) sin(obs_state(3,k,kk)+pi);cos(obs_state(3,k,kk)+pi/2) sin(obs_state(3,k,kk)+pi/2); cos(obs_state(3,k,kk)-pi/2) sin(obs_state(3,k,kk)-pi/2)];
%     g_p1{kk}(1,k)=Gp1{kk}(1,:,k)*obs_state(1:2,k,kk)+statobs(kk,4)/2;
%     g_p1{kk}(2,k)=Gp1{kk}(2,:,k)*obs_state(1:2,k,kk)+statobs(kk,4)/2;
%     g_p1{kk}(3,k)=Gp1{kk}(3,:,k)*obs_state(1:2,k,kk)+statobs(kk,5)/2;
%     g_p1{kk}(4,k)=Gp1{kk}(4,:,k)*obs_state(1:2,k,kk)+statobs(kk,5)/2;
%     movobs11{kk}(1,:,k)=(inv(Gp1{kk}([1 3],:,k))*g_p1{kk}([1 3],k))';
%     movobs11{kk}(2,:,k)=(inv(Gp1{kk}([1 4],:,k))*g_p1{kk}([1 4],k))';
%     movobs11{kk}(3,:,k)=(inv(Gp1{kk}([2 4],:,k))*g_p1{kk}([2 4],k))';
%     movobs11{kk}(4,:,k)=(inv(Gp1{kk}([2 3],:,k))*g_p1{kk}([2 3],k))';  
   if k~=1
       try
%            set(movv1{kk},'xdata',movobs11{kk}(:,1,k));
%            set(movv1{kk},'ydata',movobs11{kk}(:,2,k));
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