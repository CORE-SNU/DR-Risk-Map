figure
set(gcf,'Position',[-1923,453,1813,335]);
set(gcf,'Color','white');
axis([x_min x_max y_min y_max]);
hold on
xlabel('X', 'FontSize',16)
ylabel('Y', 'FontSize',16)
set(gca,'Position',[0.034215311131506,0.158208955223881,0.954606680584542,0.716417910447761]);
set(gca,'Color',[0.9 0.9 0.9], 'FontSize',14);
daspect([1 1 1])


% Plot the lanes
lane1=line([0 x_max],[0 0],'LineWidth',2,'Color','k');
lane2=line([0 x_max],[3.7 3.7],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane3=line([0 x_max],[7.4 7.4],'LineWidth',2,'Color','k', 'LineStyle', '--');
lane4=line([0 x_max],[y_max y_max],'LineWidth',2,'Color','k');
hold on

an1 = annotation('textbox',[0.930030995459863,0.50717023801752,0.049207405181119,0.135713935657742],'String','$y_r^{goal}$','EdgeColor','none','Interpreter','latex','FontSize',22,'FitBoxToText','on');
an2 = annotation('textbox',[0.046597212309403,0.504973729614894,0.047012060646699,0.134925369433503],'String','$y_r^{init}$','EdgeColor','none','Interpreter','latex','FontSize',22,'FitBoxToText','on');

% Plot start and goal states
plot(goalstate(1), goalstate(2),'.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);
plot(startstate(1), startstate(2), '.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);

% Plot obstacles trajectories
for io=1:size(obs_state,3)
    plot(obs_state(1,:,io),obs_state(2,:,io),'Color','b','LineStyle',':');
end

% Some initializations for rendering
rob_obj = rectangle('Position',[rob_state(1,1)-rob_rad rob_state(2,1)-rob_rad 2*rob_rad 2*rob_rad],'Curvature',[1 1],'FaceColor',[[205 16 26]/255 0.8],'EdgeColor',[[164 13 20]/255 0.8]);
new_pl{1} = plot(q_start.coord(1), q_start.coord(2), '.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);
rob_traj = plot(q_start.coord(1), q_start.coord(2), 'Color', 'k', 'LineWidth',2);
rob_pl = plot(q_start.coord(1), q_start.coord(2), '.', 'Color', 'k', 'LineWidth',2, 'MarkerSize',25);

