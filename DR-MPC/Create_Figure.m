figure
axis([0 4.3 0 5.2])
xlabel('X', 'FontSize',14)
ylabel('Y', 'FontSize',14)
daspect([1 1 1])
set(gcf,'Position',[-1294,112,732,846],'Color','w');
set(gca,'Position',[0.044687189672294,0.056737588652482,0.923535253227408,0.910767652437667]);

% Static obstacles are represented as polytopes {x | A_obs x <= b_obs^j}
A_obs = [ 1  0;
         -1  0;
          0  1;
          0 -1];

b_obs = [ 1.3   2.9   1.3   2.9   3.6   4.3;
        -0.3  -1.9  -0.3  -1.9  -0.6     0;
         1.5   1.5   3.5   3.5   5.2   5.2;
        -0.5  -0.5  -2.5  -2.5  -4.5     0];

% Extract centroid of each static obstacle
statobs = reshape([-b_obs(2,1:4)+(b_obs(1,1:4)+b_obs(2,1:4))/2;-b_obs(4,1:4)+(b_obs(3,1:4)+b_obs(4,1:4))/2],Nobs*2,1);

% Plot static obstacles
r1 = rectangle('Position',[-b_obs(2,1) -b_obs(4,1) b_obs(1,1)+b_obs(2,1) b_obs(3,1)+b_obs(4,1)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r2 = rectangle('Position',[-b_obs(2,2) -b_obs(4,2) b_obs(1,2)+b_obs(2,2) b_obs(3,2)+b_obs(4,2)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r3 = rectangle('Position',[-b_obs(2,3) -b_obs(4,3) b_obs(1,3)+b_obs(2,3) b_obs(3,3)+b_obs(4,3)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r4 = rectangle('Position',[-b_obs(2,4) -b_obs(4,4) b_obs(1,4)+b_obs(2,4) b_obs(3,4)+b_obs(4,4)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r5 = rectangle('Position',[-b_obs(2,5) -b_obs(4,5) b_obs(1,5)+b_obs(2,5) b_obs(3,5)+b_obs(4,5)], 'EdgeColor',[0.42,0.36,0.36],'FaceColor',[0.80, 0.80, 0.80],'Curvature',0.2);
r6 = rectangle('Position',[-b_obs(2,6) -b_obs(4,6) b_obs(1,6)+b_obs(2,6) b_obs(3,6)+b_obs(4,6)], 'EdgeColor', 'k','LineWidth',3);
hold on

set(gca, 'XTickLabel', cellfun(@num2str,num2cell(get(gca,'XTick')-1.7),'un',0))
set(gca, 'YTickLabel', cellfun(@num2str,num2cell(get(gca,'YTick')-0.5),'un',0))

% Plot dynamic obstacle trajectories
for i=1:L
    plot(obs_state(1,:,i),obs_state(2,:,i), 'b:');
end

% Initialize some plots
start = plot(x0(1),x0(2),'.','Color',[0.47,0.67,0.19],'MarkerSize',25);
goalpl=plot(goal(1,:),goal(2,:),'--','Color',[0.50,0.50,0.50]);
endpl = plot(goal(1,end),goal(2,end),'b.','MarkerSize',25);
fut_pl=plot(x0(1), x0(2), 'Color', 'b', 'LineWidth',1);
rob_traj=plot(x0(1), x0(2), 'Color', 'k', 'LineWidth',1);
rob_pl=rectangle('Position',[x0(1)-rob_rad, x0(2)-rob_rad, 2*rob_rad, 2*rob_rad],'EdgeColor','r','FaceColor', 'r','Curvature',1);
rob_pr=plot(x0(1), x0(2), 'Color', 'r', 'LineWidth',1);

% Start Recording
if save_video==true
    writerObj = VideoWriter(['MPC_forces_',str,'.avi']);
    writerObj.FrameRate = 10;
    writerObj.Quality=100;
    open(writerObj)
end