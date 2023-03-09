clear all
close all

Set_Params

obs_state(:,:,1) = [5 18.15]';

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

obs_traj=plot(obs_state(1,1,1),obs_state(2,1,1),'Color',[150 150 150]/255,'LineStyle',':');
obs_traj_pred=plot(obs_state(1,1,1),obs_state(2,1,1),'Color','r');


goal{2}=[42:-0.7:26 4.15*cos(-pi/2:-0.1:-pi)+26 repmat(21.85,1,length(26:0.7:80));repmat(21.85,1,length(42:-0.7:26)) 4.15*sin(-pi/2:-0.1:-pi)+26 26:0.7:80];
goal{1}=[5:0.7:16 2.15*cos(pi/2:-0.2:0)+16.5 repmat(18.65,1,length(15:-0.7:0)); repmat(18.15,1,length(5:0.7:16)) 2.15*sin(pi/2:-0.2:0)+16 15:-0.7:0];
plot(goal{1}(1,:),goal{1}(2,:),'k');
j(1) = 1;
for k=1:100
    if norm(obs_state(1:2,k,1)-[21.85;42])<=0.5
        break
    end
    if abs(obs_state(1,k,1)-17)<=0.5
        goal{1}(:,k:k+length([4.85*cos(-pi/2:0.1:0)+17 repmat(21.85,1,length(23:0.7:42))])-1)=[4.85*cos(-pi/2:0.1:0)+17 repmat(21.85,1,length(23:0.7:42)); 4.85*sin(-pi/2:0.1:0)+23 23:0.7:42];
        goal{1}(:,length([4.85*cos(-pi/2:0.1:0)+17 repmat(21.85,1,length(23:0.7:42))])+1:length([4.85*cos(-pi/2:0.1:0)+17 repmat(21.85,1,length(23:0.7:42))])+50)=repmat([21.85;42],1,50);
        plot(goal{1}(1,:),goal{1}(2,:),'k');
    end

    sol{1} = control({obs_state(1:2,k,1),goal{1}(:,k:k+K-1),10});
    obs_state(:,k+1,1) = sol{1}{1}(:,2);
    obs_input(:,k,1) = sol{1}{2}(:,1);
    set(obs_traj, 'xdata', obs_state(1,1:k+1,1), 'ydata', obs_state(2,1:k+1,1));
    set(obs_traj_pred, 'xdata', sol{1}{1}(1,:), 'ydata', sol{1}{1}(2,:));
    if k~=1
        set(obs_obj{1}, 'Position',[obs_state(1,k+1,1)-obs_rad obs_state(2,k+1,1)-obs_rad 2*obs_rad 2*obs_rad]);
    else
        obs_obj{1} = rectangle('Position',[obs_state(1,1,1)-obs_rad obs_state(2,1,1)-obs_rad 2*obs_rad 2*obs_rad],'FaceColor',[0 .5 .5 0.05],'EdgeColor',[0 .5 .5],'Curvature',[1 1]);
    end
   pause(0.1)
end
plot(obs_state(1,:,1),obs_state(2,:,1),'k');

save('obs_traj.mat', 'obs_input', 'obs_state')