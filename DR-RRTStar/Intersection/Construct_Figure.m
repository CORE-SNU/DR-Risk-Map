figure
set(gcf,'Color','white', 'Position',[-1406,36,1000,1000]);
set(gca,'Position',[0.073588709677419,0.083333333333333,0.884072580645161,0.884328358208955], 'FontSize',14);
axis([x_min x_max y_min y_max]);
hold on
xlabel('X', 'FontSize',18)
ylabel('Y', 'FontSize',18)
daspect([1 1 1])


% Plot the lanes
rectangle('Position',[0.05 0.05 39.95 39.95], 'FaceColor',[0.9 0.9 0.9],'EdgeColor','none')
rectangle('Position',[9.7 23.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[23.7 23.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[23.7 23.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[9.7 9.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)
rectangle('Position',[23.7 9.7 6.6 6.6],'Curvature',1,'FaceColor','w','EdgeColor','k','LineWidth',2)

rectangle('Position',[0.05 0.05 13 16.3],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[13 0.05 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[23.7 0.05 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[27 0.05 13 16.3],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[0.05 23.7 13 16.3],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[13 27 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[23.7 27 3.3 13],'FaceColor','w','EdgeColor','none') 
rectangle('Position',[27 23.7 13 16.3],'FaceColor','w','EdgeColor','none') 

lane1=line([0.05 13],[16.3 16.3],'LineWidth',2,'Color','k');
lane2=line([0.05 13],[20 20],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane3=line([0.05 13],[23.7 23.7],'LineWidth',2,'Color','k');
lane4=line([27 39.95],[16.3 16.3],'LineWidth',2,'Color','k');
lane5=line([27 39.95],[20 20],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane6=line([27 39.95],[23.7 23.7],'LineWidth',2,'Color','k');
lane7=line([16.3 16.3],[0.05 13],'LineWidth',2,'Color','k');
lane8=line([20 20],[0.05 13],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane9=line([23.7 23.7],[0.05 13],'LineWidth',2,'Color','k');
lane10=line([16.3 16.3],[27 39.95],'LineWidth',2,'Color','k');
lane11=line([20 20],[27 39.95],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane12=line([23.7 23.7],[27 39.95],'LineWidth',2,'Color','k');

hold on
an1 = annotation('textbox',[0.080866068528764,0.569000000000001,0.047133931471236,0.032093128899142],'String','$y_r^{goal}$','EdgeColor','none','Interpreter','latex','FontSize',22,'FitBoxToText','on');
an2 = annotation('textbox',[0.518683725876434,0.082426801290248,0.071554163052018,0.038799999332428],'String','$y_r^{init}$','EdgeColor','none','Interpreter','latex','FontSize',22,'FitBoxToText','on');

% Plot start and goal states
plot(goalstate(1), goalstate(2),'.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);
plot(startstate(1), startstate(2), '.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);

% Plot obstacle trajectories
for io=1:size(obs_state,3)
    plot(obs_state(1,:,io),obs_state(2,:,io),'Color',[150 150 150]/255,'LineStyle',':');
end

% Some initializations for rendering
rob_obj=rectangle('Position',[rob_state(1,1)-rob_rad rob_state(2,1)-rob_rad 2*rob_rad 2*rob_rad],'Curvature',[1 1],'FaceColor',[[205 16 26]/255 0.8],'EdgeColor',[[164 13 20]/255 0.8]);
new_pl{1}=plot(q_start.coord(1), q_start.coord(2), '.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);
rob_traj=plot(q_start.coord(1), q_start.coord(2), 'Color', 'k', 'LineWidth',2);
rob_pl=plot(q_start.coord(1), q_start.coord(2), '.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);

