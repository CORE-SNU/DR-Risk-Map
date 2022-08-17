close all
clear all
global ucost L W minAngle angIncrement maxAngle minVel minW maxW delta velIncrement maxVel Ts wIncrement root new_pl vels angles poscont w q_goal K r2 controller steercontrol t xy21 xy21_var;
L=2; W=0.2; Ts=0.1;
load('Dynamics.mat')
K=15;
M1=50;
figure
set(gcf,'Position',[122,138,876,548]);
set(gca,'Position',[0.067921717609597,0.080202455622019,0.909090909090908,0.953890489913545],'FontSize',18);
hold on
xlabel('X','FontSize',25)
ylabel('Y', 'FontSize',25)
obs = 0;
obs_state = [0.18;0.18;pi/12];
% obs_input = [linspace(2,4,11) linspace(3,4,10) linspace(4,3,10) repmat(3,1,20); repmat(0,1,1) repmat(1.2,1,5) repmat(0,1,2) repmat(-1.6,1,5) repmat(0,1,3) repmat(1,1,5) repmat(-0.4,1,5) repmat(-2,1,5) repmat(-1,1,3) repmat(0,1,5) repmat(0,1,12)];
% obs_input = [linspace(3,4,11) linspace(4,4,10) repmat(3,1,20); repmat(0,1,1) repmat(0.5,1,4) repmat(0,1,3) repmat(0,1,2) repmat(1.6,1,3) repmat(0,1,3) repmat(-1.2,1,15) repmat(1,1,10)];
obs_input = [linspace(3,4,11) linspace(4,4,10) repmat(4,1,20); repmat(0,1,1) repmat(0.5,1,4) repmat(0,1,3) repmat(1.6,1,2) repmat(0.5,1,6) repmat(-1.2,1,15) repmat(0,1,10)];

meanfunc = {@meanZero};
covfunc = {@covSEiso};
likfunc ={@likGauss};
hyp = struct('mean', [] , 'cov', [0 0], 'lik', -1);

writerObj = VideoWriter(['Simulations/GP.avi']);
writerObj.FrameRate = 10;
writerObj.Quality=100;
open(writerObj)

obs_state = vehicledyn(obs_input',obs_state',1);
axis([0 max(obs_state(1,:))-0.5 0 max(obs_state(2,:))+(max(obs_state(2,:))-min(obs_state(2,:)))/3]);
daspect([1 1 1])
% obs2 = obs_state(:,1)';
% for t=1:110
% obs2(t+1,:) = vehicledyn_NN(obs_input(:,t)',obs2(t,:),w,b,mean_train,std_train);
% end
act = scatter(obs_state(1,:),obs_state(2,:),300,'MarkerFaceColor',[0.15,0.73,0.15],'MarkerEdgeColor',[0.14,0.72,0.14],'MarkerFaceAlpha',0.6,'MarkerEdgeAlpha',1);
stat = scatter(obs_state(1,1),obs_state(2,1),300,'MarkerEdgeColor',[0.66,0.06,0.21],'MarkerFaceColor',[0.82,0.11,0.24],'MarkerEdgeAlpha',1);
plot(obs_state(1,:),obs_state(2,:),'k', 'Marker','.','MarkerSize',8)

% pred2 = plot(obs_state(1,1),obs_state(2,1),'b','LineWidth',2);

for t=1:41
    GP_pred1
    %GP_pred_NN
    set(stat,'xdata',obs_state(1,t),'ydata',obs_state(2,t));
    uistack(movv1{1},'top')
    uistack(movv11{1},'top')
    %uistack(movv2{1},'top')
    uistack(stat,'top')
    %uistack(movv21{1},'top')
%     legend([movv1{1} movv2{1} act], 'GPR', 'GPR + NN','Actual','location','northwest','interpreter','latex','FontSize',25);
    [~, objh] = legend([movv1{1} act], 'GPR','Actual','location','northwest','interpreter','latex','FontSize',25);    
    objh1  = findobj(objh,'type','patch')
    set(objh1(1),'MarkerSize',12)
    set(objh1(2),'MarkerSize',12)
%     saveas(gcf,['Simulations/GP_',num2str(t-1)],'epsc');
%     saveas(gcf,['Simulations/GP_',num2str(t-1)],'png');
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
%     if t==4 || t==21 || t==30
%         for kk=1:size(obs,1)
%             movv1_1{kk}=scatter(xy21{t}(:,1,kk),xy21{t}(:,2,kk),300,'MarkerFaceAlpha',0.8,'MarkerEdgeAlpha',1,'MarkerFaceColor',[0.06,0.45,0.72],'MarkerEdgeColor',[0.12,0.12,0.62]);
%             movv11_1{kk}=plot(xy21{t}(:,1,kk),xy21{t}(:,2,kk),'k', 'Marker','.','MarkerSize',8);
%         end
%         stat1 = scatter(obs_state(1,t),obs_state(2,t),300,'MarkerEdgeColor',[0.66,0.06,0.21],'MarkerFaceColor',[0.82,0.11,0.24],'MarkerEdgeAlpha',1);
%     end
end
close(writerObj)