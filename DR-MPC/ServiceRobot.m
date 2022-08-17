clear all
close all
clc
figure
axis([0 10 0 10])
daspect([1 1 1])
set(gcf,'Position',[0,36,954,954],'Color','w');
set(gca,'Position',[0.044687189672294,0.048218029350105,0.923535253227408,0.919287211740044]);

global Ts A1 b1
veh_rad = 0.1;
K=5;
Ts=0.1;
nx=4;
ny=2;
nu=2;
A1 = [1 0; -1 0; 0 1; 0 -1];
b1(:,1) = [3; 0; 10; -3];
b1(:,2) = [3; 0; 1; 0];
b1(:,3) = [5.5; -4.5; 2.5; -1.5];
% b1(:,4) = [8; -7; 2.5; -1.5];
b1(:,4) = [7.1; -6.1; 2.5; -1.5];
b1(:,5) = [5.5; -4.5; 4.5; -3.5];
% b1(:,6) = [8; -7; 4.5; -3.5];
b1(:,6) = [7.1; -6.1; 4.5; -3.5];
b1(:,7) = [7.75; -4.75; 9; -6];
b1(:,8) = [10; -9; 7; -5];
b1(:,9) = [10; 0; 10; 0];

r1 = rectangle('Position',[-b1(2,1) -b1(4,1) b1(1,1)+b1(2,1) b1(3,1)+b1(4,1)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80]);
r2 = rectangle('Position',[-b1(2,2) -b1(4,2) b1(1,2)+b1(2,2) b1(3,2)+b1(4,2)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80]);
r3 = rectangle('Position',[-b1(2,3) -b1(4,3) b1(1,3)+b1(2,3) b1(3,3)+b1(4,3)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r4 = rectangle('Position',[-b1(2,4) -b1(4,4) b1(1,4)+b1(2,4) b1(3,4)+b1(4,4)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r5 = rectangle('Position',[-b1(2,5) -b1(4,5) b1(1,5)+b1(2,5) b1(3,5)+b1(4,5)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r6 = rectangle('Position',[-b1(2,6) -b1(4,6) b1(1,6)+b1(2,6) b1(3,6)+b1(4,6)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r7 = rectangle('Position',[-b1(2,7) -b1(4,7) b1(1,7)+b1(2,7) b1(3,7)+b1(4,7)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r8 = rectangle('Position',[-b1(2,8) -b1(4,8) b1(1,8)+b1(2,8) b1(3,8)+b1(4,8)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r9 = rectangle('Position',[-b1(2,9) -b1(4,9) b1(1,9)+b1(2,9) b1(3,9)+b1(4,9)], 'EdgeColor','k','LineWidth',3);
hold on

x0(:,1) = [1;2;0;0];
goal = [1 4 5.8 5.8 5.8 7.5 8.4 8.4 9; 2 1.1 1.1 2 5 5 5 9 9];
req_t = Required_Time(goal,0.5);
ts=timeseries(goal',req_t);
ref = resample(ts,0:Ts:req_t(end));
ref1 = ref.data';
ref1(:,end+1:end+K) = repmat(goal(:,end),1,K);

start = plot(x0(1,1),x0(2,1),'.','Color',[0.47,0.67,0.19],'MarkerSize',25);
goalpl=plot(goal(1,:),goal(2,:),'--','Color',[0.50,0.50,0.50]);
endpl = plot(goal(1,end),goal(2,end),'b.','MarkerSize',25);
fut_pl=plot(x0(1,1), x0(2,1), 'Color', 'b', 'LineWidth',1);
veh_traj=plot(x0(1,1), x0(2,1), 'Color', 'k', 'LineWidth',1);
veh_pl=rectangle('Position',[x0(1,1)-veh_rad, x0(2,1)-veh_rad, 2*veh_rad, 2*veh_rad],'EdgeColor','r','FaceColor', 'r','Curvature',1);
veh_pr=plot(x0(1,1), x0(2,1), 'Color', 'r', 'LineWidth',1);


% % % x_min = 0; x_max = 10; y_min = 0; y_max = 10; vx_min = -4; vx_max = 4; vy_min = -4; vy_max = 4; ax_min = -4; ax_max = 4; ay_min = -4; ay_max = 4;
% % % accx=ax_min:0.02:ax_max;
% % % accy=ay_min:0.02:ay_max;
% % % poscont=combvec(accx,accy);

% % % ref = x0(:,1);
% % % jg = 1;
% % % for jg = 1:length(goal)
% % %     ref = [ref RRT_Star(ref(:,end),[goal(:,jg);0;0])];
% % % end

Q=eye(ny);
R=0.001*eye(nu);
str = '0_05';
load(['Data_',str]);
L=3;

obs = [3.5 9 9.17;5.5 9.7 9.5];
u_obs(:,:,1)=[repmat([0.35;0],1,65) repmat([0;-0.4],1,30) repmat([0;-0.5],1,50) repmat([0;-0.4],1,1000-145)];
u_obs(:,:,2)=[repmat([0;0],1,84) repmat([-0.05;-0.3],1,123) repmat([-0.4;-0.2],1,55) repmat([-0.2;0],1,77) repmat([0;0],1,1000-339)];
u_obs(:,:,3)=[repmat([0;0],1,84) repmat([-0.05;-0.3],1,123) repmat([-0.4;-0.2],1,55) repmat([-0.2;0],1,77) repmat([0;0],1,1000-339)];
% u_obs(:,:,1)=[repmat([0.35;0],1,32) repmat([0;-0.6],1,15) repmat([0;-0.7],1,25) repmat([0;-0.4],1,1000-72)];
% u_obs(:,:,2)=[repmat([0;0],1,42) repmat([-0.05;-0.3],1,61) repmat([-0.4;-0.2],1,28) repmat([-0.2;0],1,39) repmat([0;0],1,1000-170)];
% u_obs(:,:,3)=[repmat([0;0],1,42) repmat([-0.05;-0.3],1,61) repmat([-0.4;-0.2],1,28) repmat([-0.2;0],1,39) repmat([0;0],1,1000-170)];
W=0.1;

for i=1:size(obs,2)
    obs_state(:,:,i) = obstacledyn(obs(:,i),u_obs(:,:,i),Ts,1000);
    obs_input(:,:,i) = u_obs(:,:,i);
%     obs1{i} = rectangle('Position',[obs(1,i)-W obs(2,i)-W 2*W 2*W],'EdgeColor','k','FaceColor','k','Curvature',1);
    plot(obs_state(1,:,i),obs_state(2,:,i), 'b:');
end

r=sdpvar(ny,K+1,'full');
u=sdpvar(nu,K,'full');
x=sdpvar(nx,K+1,'full');
d=binvar(4,K,length(b1)-7,'full');
lambda1=sdpvar(Nn,K,L,'full');
lambda2=sdpvar(Nn,K,L,'full');
lambda3=sdpvar(Nn,K,L,'full');
d1=binvar(Nn,K,L,'full');
d2=binvar(Nn,K,L,'full');
d3=binvar(Nn,K,L,'full');
h1=sdpvar(Nn,K,L,'full');
h2=sdpvar(Nn,K,L,'full');
h3=sdpvar(Nn,K,L,'full');
h4=sdpvar(K,L,'full');
mu_hat=sdpvar(ny,K,L,'full');
Sigma_hat=sdpvar(ny,ny,K,L,'full');
rho=sdpvar(4,K,length(b1)-5,'full');
% delta=sdpvar(L,1,'full');
delta=0.1/100;

M1=100;
M=1000;
obj=0;
const=[];
for k=1:K
    if k>0 && k<=K
        obj=obj+u(:,k)'*R*u(:,k);
%         const=const+[-1<=x(4,k)-x(4,k-1)<=1,...
%                     -0.7<=u(2,k)-u(2,k-1)<=0.7,...
%                     ];
    end
    obj=obj+(x(1:2,k)-r(:,k))'*Q*(x(1:2,k)-r(:,k));
    const=const+[...
%           x(1,k+1)==x(1,k)+Ts*x(4,k)*cos(x(3,k)),...
%           x(2,k+1)==x(2,k)+Ts*x(4,k)*sin(x(3,k)),...
%           x(3,k+1)==x(3,k)+Ts*u(2,k),...
%           x(4,k+1)==x(4,k)+Ts*u(1,k),...
%           -4<=u(1,k)<=4,...
        x(1,k+1)==x(1,k)+Ts*x(3,k)+Ts^2/2*u(1,k),...
        x(2,k+1)==x(2,k)+Ts*x(4,k)+Ts^2/2*u(2,k),...
        x(3,k+1)==x(3,k)+Ts*u(1,k),...
        x(4,k+1)==x(4,k)+Ts*u(2,k),...
         -4<=x(3,k)<=4,...
         -4<=x(4,k)<=4,...
         -4<=u(1,k)<=4,...
         -4<=u(2,k)<=4,...
%          -3.15<=u(2,k)<=3.15,...
         ];
     for i=1:length(b1)-7
        const=const+[b1(:,i+4)+0.1-A1*x(1:2,k+1)<=M1*d(:,k,i),sum(d(:,k,i))<=3];
     end
% for i=1:length(b1)-1
%     const=const+[rho(:,k,i)'*(b1(:,i)+0.15-A1*x(1:2,k+1))<=0,rho(:,k,i)>=0,sum(rho(:,k,i))==1];
% end
    for l=1:L
                 const=const+[
                 h1(:,k,l)>=0, h1(:,k,l)>=w{1}'*(([x(1:2,k+1);Sigma_hat(1,1,k,l);Sigma_hat(1,2,k,l);...
                              Sigma_hat(2,2,k,l);mu_hat(:,k,l)]-mean_train')./std_train')+b{1}',...
                 h1(:,k,l)<=M*d1(:,k,l), h1(:,k,l)<=w{1}'*(([x(1:2,k+1);Sigma_hat(1,1,k,l);Sigma_hat(1,2,k,l);...
                              Sigma_hat(2,2,k,l);mu_hat(:,k,l)]-mean_train')./std_train')+b{1}'+M*(1-d1(:,k,l)),...                              
                 h2(:,k,l)>=0, h2(:,k,l)>=w{2}'*h1(:,k,l)+b{2}',...
                 h2(:,k,l)<=M*d2(:,k,l), h2(:,k,l)<=w{2}'*h1(:,k,l)+b{2}'+M*(1-d2(:,k,l)),...
                 h3(:,k,l)>=0, h3(:,k,l)>=w{3}'*h2(:,k,l)+b{3}',...
                 h3(:,k,l)<=M*d3(:,k,l), h3(:,k,l)<=w{3}'*h2(:,k,l)+b{3}'+M*(1-d3(:,k,l)),...
                 w{4}'*h3(:,k,l)+b{4}'<=delta]; 
%                  h4(k,l)=subplus((w{4}'*h3(:,k,l)+b{4}'));              
%                  const=const+[
%                  h1(:,k,l)>=0, h1(:,k,l)-w{1}'*(([x(1:2,k+1);Sigma_hat(1,1,k,l);Sigma_hat(1,2,k,l);...
%                             Sigma_hat(2,2,k,l);mu_hat(:,k,l)]-mean_train')./std_train')-b{1}'>=0,...
%                  h2(:,k,l)>=0, h2(:,k,l)-w{2}'*h1(:,k,l)-b{2}'>=0,...
%                  h3(:,k,l)>=0, h3(:,k,l)-w{3}'*h2(:,k,l)-b{3}'>=0,...
%                  (w{4}'*h3(:,k,l)+b{4}')<=delta(l),...
%                  h1(:,k,l)'*(h1(:,k,l)-(w{1}'*(([x(1:2,k+1);Sigma_hat(1,1,k,l);Sigma_hat(1,2,k,l);...
%                              Sigma_hat(2,2,k,l);mu_hat(:,k,l)]-mean_train')./std_train')+b{1}'))==0,...
%                  h2(:,k,l)'*(h2(:,k,l)-w{2}'*h1(:,k,l)-b{2}')==0,...
%                  h3(:,k,l)'*(h3(:,k,l)-w{3}'*h2(:,k,l)-b{3}')==0,...
%                  ];
%                  h4(k,l)=(w{4}'*h3(:,k,l)+b{4}');
%                 h4(k,l)=(subplus(subplus(subplus((([x(1:2,k+1);Sigma_hat(1,1,k,l);Sigma_hat(1,2,k,l);...
%                               Sigma_hat(2,2,k,l);mu_hat(:,k,l)]'-mean_train)./std_train)*w{1}+b{1})*w{2}+b{2})*w{3}+b{3})*w{4}+b{4});
%                 const=const+[(subplus(subplus(subplus((([x(1:2,k+1);Sigma_hat(1,1,k,l);Sigma_hat(1,2,k,l);...
%                               Sigma_hat(2,2,k,l);mu_hat(:,k,l)]'-mean_train)./std_train)*w{1}+b{1})*w{2}+b{2})*w{3}+b{3})*w{4}+b{4})<=delta];
    end
end
obj=obj+(x(1:2,K+1)-r(:,K+1))'*Q*(x(1:2,K+1)-r(:,K+1));

opt=sdpsettings('solver','gurobi','gurobi.Method',3,'gurobi.Presolve',2,'gurobi.MIPFocus',1,'gurobi.TimeLimit',20);
controller=optimizer(const,obj,opt,{x(:,1),Sigma_hat,mu_hat,r},{u,x,obj,h4});
% 
meanfunc = {@meanZero};
covfunc = {@covSEiso};
likfunc ={@likGauss};
hyp = struct('mean', [] , 'cov', [0 0], 'lik', -1);

writerObj = VideoWriter(['MPC_',str,'.avi']);
% writerObj = VideoWriter(['MPC_0_00005.avi']);
writerObj.FrameRate = 10;
writerObj.Quality=100;
open(writerObj)
t=0;
cost=[];
terminate=0;
% ref1 = waypoints(:,1);
solve_time=tic;
while ~terminate
    t=t+1;
    set(veh_traj, 'xdata', x0(1,1:t),'ydata', x0(2,1:t));
    set(veh_pl,'Position',[x0(1,t)-veh_rad, x0(2,t)-veh_rad, 2*veh_rad, 2*veh_rad]);
%     delete([veh_r veh_l]);
%     R=[cos(x0(3,t)) -sin(x0(3,t));sin(x0(3,t)) cos(x0(3,t))];
%     p_r=R*[-r1 r1 r1 -r1;-r2 -r2 r2 r2]+[x0(1,t)+veh_rad*cos(x0(3,t)-pi/2);x0(2,t)+veh_rad*sin(x0(3,t)-pi/2)];
%     p_l=R*[-r1 r1 r1 -r1;-r2 -r2 r2 r2]+[x0(1,t)+veh_rad*cos(x0(3,t)+pi/2);x0(2,t)+veh_rad*sin(x0(3,t)+pi/2)];
%     veh_r=fill(p_r(1,:),p_r(2,:),'k');
%     veh_l=fill(p_l(1,:),p_l(2,:),'k');
%     for i=1:size(obs,2)
%         set(obs1{i},'Position',[obs_state(1,t,i)-W obs_state(2,t,i)-W 2*W 2*W]);
%     end
    GP_pred
%     if norm(x0(1:2,t)-future_r)<=0.001
%         refn = refn+1;
%         if refn > size(waypoints,2)
%             break;
%         else
%             future_r = waypoints(:,refn);
%         end
%     end
    if t==1
%     future_r=ref1;
    oldrefnum=0;
    refnum=1;
    set(fut_pl,'xdata',ref1(1,t:t+K),'ydata',ref1(2,t:t+K));
    else
        if refnum<size(ref1,2)-10
%            ref1pl = plot(ref1(1,refnum+1:end),ref1(2,refnum+1:end),'m');
            oldrefnum=refnum;
            dist=vecnorm(x0(1:2,t)-ref1(:,oldrefnum:oldrefnum+10));
            [~,refnum]=min(dist);
            refnum = oldrefnum+refnum-1;
            set(fut_pl,'xdata',ref1(1,refnum+1:refnum+K),'ydata',ref1(2,refnum+1:refnum+K));
        else
%            ref1pl = plot(ref1(1,refnum+1:end),ref1(2,refnum+1:end),'m');
            oldrefnum=refnum;
            dist=vecnorm(x0(1:2,t)-ref1(:,oldrefnum:end));
            [~,refnum]=min(dist);
            refnum = oldrefnum+refnum-1;
            set(fut_pl,'xdata',ref1(1,refnum+1:refnum+K),'ydata',ref1(2,refnum+1:refnum+K));
        end
    end
    if refnum+K+1>size(ref1,2)
      ref1(:,refnum+K+1) = ref1(:,refnum+K);
    end
%     set(fut_pl,'xdata',ref1(1,t:t+K),'ydata',ref1(2,t:t+K));
    j = [];
    if norm(x0(1:2,t)-obs_state(:,t,1))<=4
        j=[j 1];
    end
    if norm(x0(1:2,t)-obs_state(:,t,2))<=4
        j=[j 2];
    end
    if norm(x0(1:2,t)-obs_state(:,t,3))<=4
        j=[j 3];
    end
    if t>=220
        j=[];
    end
%     delta1(1:length(j)) =0.1/100;
    %0.1/100;
%     delta1(length(j)+1:L) = 1000;
%     var_in(:,:,:,1:length(j)) = xy21_var{t}(:,:,2:end,j);
%     var_in(:,:,:,length(j)+1:L) = zeros(ny,ny,K,L-length(j));
%     mu_in(:,:,1:length(j)) = xy21{t}(:,2:end,j);
%     mu_in(:,:,length(j)+1:L) = zeros(ny,K,L-length(j));
    var_in(:,:,:,1:L) = xy21_var{t}(:,:,2:end,:);
    mu_in(:,:,1:L) = xy21{t}(:,2:end,:);
    [sol,diag1{t}]=controller({x0(:,t),var_in,mu_in,ref1(:,refnum+1:refnum+K+1)});
%     solvertime(t)=gurobiout.solvertime;
    U{t} = sol{1};
    Xout{t}= sol{2};
    DR_risk(:,:,t)=sol{4};
    if diag1{t}~=0 && diag1{t}~=3
        error(yalmiperror(diag1{t}));
    end
    x0(1,t+1)=x0(1,t)+Ts*x0(3,t)+Ts^2/2*U{t}(1,1);
    x0(2,t+1)=x0(2,t)+Ts*x0(4,t)+Ts^2/2*U{t}(2,1);
    x0(3,t+1)=x0(3,t)+Ts*U{t}(1,1);
    x0(4,t+1)=x0(4,t)+Ts*U{t}(2,1);
%     x0(1,t+1)=x0(1,t)+Ts*x0(4,t)*cos(x0(3,t));
%     x0(2,t+1)=x0(2,t)+Ts*x0(4,t)*sin(x0(3,t));
%     x0(3,t+1)=x0(3,t)+Ts*U{t}(2,1);
%     x0(4,t+1)=x0(4,t)+Ts*U{t}(1,1);
%     x0(1,t+1)=x0(1,t)+Ts*U{t}(1,1);
%     x0(2,t+1)=x0(2,t)+Ts*U{t}(2,1);
%     x0(3,t+1)=x0(3,t)+Ts*U{t}(3,1);
    cost=[cost sol{3}];
    set(veh_pr,'xdata',Xout{t}(1,:),'ydata',Xout{t}(2,:));
    drawnow
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
    if norm(x0(1:2,t+1)-goal(:,end))<=0.05
        set(veh_traj, 'xdata', x0(1,1:t+1),'ydata', x0(2,1:t+1));
        set(veh_pl,'Position',[x0(1,t+1)-veh_rad, x0(2,t+1)-veh_rad, 2*veh_rad, 2*veh_rad]);
        frame = getframe(gcf);
        writeVideo(writerObj, frame);
        terminate=1;
    end
%     if t~=1
%         delete(ref1pl);
%     end
end
comp_time=toc(solve_time);
close(writerObj);    
save(str);
