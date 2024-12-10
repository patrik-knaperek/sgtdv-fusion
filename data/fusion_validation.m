filename = 'fusion_data_sim';
camera_data = load(strcat([filename, '_camera.csv']));
lidar_data = load(strcat([filename, '_lidar.csv']));
%simple_fusion_data = load(strcat(['fusion_data/', filename, '_simple_fusion.csv']));
fusion_data = load(strcat([filename, '_fusion.csv']));
map_data = load(strcat([filename, '_map.csv']));
%map_data = map_data(:,1:end-7);
% map_data = load('straight_track.csv');
%map_data = load('fusion_data/V_track_1.csv');
%camera_data = camera_data(1:end-2,:);
%lidar_data = lidar_data(1:end  -2,:);
%fusion_data = fusion_data(1:end-2,:);

data_N = size(fusion_data,1)/2;
cones_N = size(map_data,2);

[camera_points, camera_rms, camera_rms_mean, ~] = processDataFunc(camera_data, data_N, map_data, cones_N);
[lidar_points, lidar_rms, lidar_rms_mean, ~] = processDataFunc(lidar_data, data_N, map_data, cones_N);
%[simple_fusion_points, simple_fusion_rms, simple_fusion_rms_mean, ~] = processDataFunc(simple_fusion_data, data_N, map_data, cones_N);
[fusion_points, fusion_rms, fusion_rms_mean, fusion_variance] = processDataFunc(fusion_data, data_N, map_data, cones_N);

% plot
blue_dark = [0.00, 0.35, 0.64];
blue_light = [0.17, 0.52, 0.70];
red_dark = [0.75,0.23,0.10];
red_light = [0.99,0.41,0.26];
purple_dark = [0.49,0.18,0.56];
purple_light = [0.62,0.27,0.80];
yellow_dark = [0.93,0.69,0.13];
yellow_light = [0.93,0.83,0.13];
green_dark = [0.0, 0.64, 0.35];
bg = [0.94,0.94,0.94];

font_name = 'Liberation Serif';
font_size = 18;


figure_pos = [2573, 525, 1200, 600];
% map
f1 = figure(1);
f1.Position = figure_pos;

ax1 = subplot(1,2,1);
ax1.PositionConstraint = 'outerposition';
ax1.Position = ax1.Position + [-0.06 0.01 0.165 -0.0];

%{
p1 = plot(ax1,map_data(2,:), map_data(1,:), 'o',...
    camera_points(2,:), camera_points(1,:), '+',...
    lidar_points(2,:), lidar_points(1,:), 'o',...
    simple_fusion_points(2,:), simple_fusion_points(1,:), '*',...
    fusion_points(2,:), fusion_points(1,:), '*',...
     'MarkerSize',15, 'LineWidth',1.5);
p1(2).MarkerEdgeColor = blue_dark;
p1(2).MarkerSize = 19;
p1(3).MarkerEdgeColor = red_dark;
p1(4).MarkerEdgeColor = purple_dark;
p1(5).MarkerEdgeColor = yellow_dark;
p1(1).MarkerFaceColor = 'black';
p1(1).MarkerSize = 8;
p1(1).MarkerEdgeColor = 'none';
%}
p1 = plot(ax1,map_data(2,:), map_data(1,:), 'o',...
    camera_points(2,:), camera_points(1,:), '+',...
    lidar_points(2,:), lidar_points(1,:), 'o',...
    fusion_points(2,:), fusion_points(1,:), '*',...
     'MarkerSize',15, 'LineWidth',1.5);
p1(2).MarkerEdgeColor = blue_dark;
p1(2).MarkerSize = 19;
p1(3).MarkerEdgeColor = red_dark;
% p1(4).MarkerEdgeColor = yellow_dark;
p1(1).MarkerFaceColor = 'black';
p1(1).MarkerSize = 8;
p1(1).MarkerEdgeColor = 'none';

hold on;

%for i = 1:size(fusion_points,2)
%    e = error_ellipse([fusion_variance(2,i), 0; 0, fusion_variance(1,i)],...
%    'mu',[fusion_points(2,i), fusion_points(1,i)]);
%    e.Color = green_dark;
%    e.LineWidth = 1;
%end

%plot(ax1, [], [], 'LineWidth', 2);

% fssim
%axis(ax1, [ -1.5, 2,0, 12]);
% straight track
axis(ax1, [-0.5+min(map_data(2,:)), 1+max(map_data(2,:)),0, 2+max(map_data(1,:))]);
% V track
%axis(ax1, [-0.5+min(map_data(2,:)), 0.5+max(map_data(2,:)), -0.5+min(map_data(1,:)), 0.5+max(map_data(1,:))]);


ax1.FontName = font_name;
ax1.FontSize = font_size;

ax1.Box ='off';
ax1.TickDir = 'both';
%ax1.XTick = min(map_data(2,:)):1:max(map_data(2,:));
%ax1.YTick = min(map_data(1,:)):1:max(map_data(1,:));
%ax1.YTick = 0:1:max(map_data(1,:))+1;
%ax1.XTickLabel = replace(ax1.XTickLabel, '.', ',');
%ax1.YTickLabel = replace(ax1.YTickLabel, '.', ',');
ax1.XDir = 'reverse';
grid on;


x1 = xlabel(ax1, '$y\ [\mathrm{m}]$', 'FontSize', font_size, 'Interpreter','latex');
y1 = ylabel(ax1, '$x\ [\mathrm{m}]$', 'FontSize', font_size, 'Interpreter','latex');
%l1 = legend(ax1, "real position", "stereocamera", "lidar", "simple fusion", "fusion with KF");
l1 = legend(ax1, "real position", "stereocamera", "lidar","fusion with KF");
l1.Location = 'eastoutside';
l1.FontName = font_name;
l1.FontSize = font_size;
l1.Orientation = 'vertical';
%l1.NumColumns = 2;
l1.Box = 'off';
l1.Color = bg;
%tl1 = title(l1, "← TRACK");

% RMSE
%f2 = figure(2)65
%f2.Position = figure_pos;

hold off;

ax2 = subplot(1,2,2);
ax2.PositionConstraint = 'outerposition';
ax2.Position = ax2.Position + [0.08 0.0 -0.06 0.0];

%bar_y = [camera_rms_mean,lidar_rms_mean,simple_fusion_rms_mean, fusion_rms_mean];
bar_y = [camera_rms_mean,lidar_rms_mean, fusion_rms_mean];
p2 = bar(ax2, bar_y, 'grouped', ...
    'EdgeColor', 'none', 'BarWidth', 0.6);
p2.FaceColor = 'flat';
p2.CData(1,:) = blue_dark;
p2.CData(2,:) = red_dark;
%p2.CData(3,:) = purple_dark;
%p2.CData(4,:) = yellow_dark;
p2.CData(3,:) = yellow_dark;

xtips1 = p2.XEndPoints;
ytips1 = p2.YEndPoints;
labels1 = replace(string(round(p2.YData,4)), '.',',');
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom', 'FontSize', 14, 'FontName',font_name);

hold on;
%p3 = plot(ax2, [0,data_N+1], [camera_rms_mean, camera_rms_mean], '--',...
%     [0,data_N+1],[lidar_rms_mean, lidar_rms_mean], '--',...
%     [0,data_N+1], [simple_fusion_rms_mean, simple_fusion_rms_mean], '--', ...
%     [0,data_N+1], [fusion_rms_mean, fusion_rms_mean], '--',...
%     'LineWidth', 3);
%p3(1).Color = blue_light;
%p3(2).Color = red_light;
%p3(3).Color = purple_light;
%p3(4).Color = yellow_light;
%axis(ax2, [0, data_N + 1, 0, 1.02*max(fusion_rms)]);

ax2.YLim(1) = 0.8 * min(bar_y);
ax2.YLim(2) = 1.05 * max(bar_y);

ax2.FontName = font_name;
ax2.FontSize = font_size;
ax2.Box = 'off';
ax2.XTick = [];
ax2.TickDir = 'both';
%ax2.XTick = 1:1:data_N;
%ax2.XTickLabel = camera_data(1:2:end,1);
%ax2.XTickLabel = replace(string(ax2.XTickLabel), '0', ' ');
ax2.YTickLabel = replace(string(ax2.YTick), '.', ',');
%ax2.XTickLabel = [];
ax2.YGrid = 'on';
ax2.YMinorGrid = 'on';



%x2 = xlabel(ax2, 'detekcie kužeľov');
%x2.FontName = font_name;
%x2.FontSize = font_size;
y2 = ylabel(ax2, "$\overline{RMSE}\ [\mathrm{m}]$");
y2.FontName = font_name;
y2.FontSize = font_size;
y2.Interpreter = 'latex';
y2.Position(2) = ax2.YLim(2)*1.006;
y2.HorizontalAlignment = 'left';
y2.Rotation = 0;

%{
t1 = text(ax2,ax2.XLim(2)*1.02, simple_fusion_rms_mean+0.002,...
replace(string(round(simple_fusion_rms_mean,4)), ".", ","));
t1.FontName = font_name;
t1.FontSize = font_size;
t1.LineWidth = 2;
t1.EdgeColor = purple_dark;
t1.Margin = 1.5;
if simple_fusion_rms_mean < fusion_rms_mean
    t1.VerticalAlignment = 'cap';
else
    t1.VerticalAlignment = 'baseline';
end

t2 = text(ax2,ax2.XLim(2)*1.02, fusion_rms_mean-0.002, ...
    replace(string(round(fusion_rms_mean,4)), ".", ","));
t2.FontName = font_name;
t2.FontSize = font_size;
t2.LineWidth = 2;
t2.EdgeColor = yellow_dark;
t2.Margin = 1.5;
if fusion_rms_mean < simple_fusion_rms_mean
    t2.VerticalAlignment = 'cap';
else
    t2.VerticalAlignment = 'baseline';
end

l2 = legend(ax2, "kamera", "LIDAR", "jednoduchá fúzia", "fúzia s KF");
%l2.Location = 'northeastoutside';
l2.Location = 'southoutside';
l2.FontName = font_name;
l2.FontSize = font_size;
l2.Box = 'off';
l2.Color = bg;
l2.Orientation = ['horizontal'];
%}

hold off
%% export figure
%exportgraphics(f1,strcat(['Zdroje/Obrazky/fusion_experiment/',filename,'_en.png']),'Resolution',300)
exportgraphics(f1,strcat([filename,'.pdf']),'ContentType', 'vector');
%exportgraphics(f2,strcat([filename,'_rms.png']), 'Resolution',300)
close(f1);

%% export data
disp(filename)
fprintf("%.2f, %.2f, %.2f\n",camera_rms_mean/fusion_rms_mean, lidar_rms_mean/fusion_rms_mean, simple_fusion_rms_mean/fusion_rms_mean)

%% error ellipse example

idx = 4;
data = camera_data;
x = data(idx*2-1,2:(1+data(idx*2-1,1)));
y = data(idx*2,2:(1+data(idx*2,1)));
mean_x = mean(x);
mean_y = mean(y);
cov_xx = sum((x - mean_x).^2) / data(idx*2,1);
cov_yy = sum((y - mean_y).^2) / data(idx*2,1);

obs = [x;y]';
mean_vec = mean(obs);
cov_mat = (obs - mean_vec)' * (obs - mean_vec) / data(idx*2,1);

f = figure(3);

s = scatter(y, x);
s.MarkerFaceColor = blue_dark;
hold on;
s_mu = scatter(mean_vec(2), mean_vec(1));
s_mu.MarkerFaceColor = red_dark;
s_mu.MarkerEdgeColor = 'none';
s_mu.SizeData = 100;

e = error_ellipse([cov_mat(4), 0; 0, cov_mat(1)],...
    'mu',[mean_vec(2), mean_vec(1)]);
    e.Color = red_dark;
    e.LineWidth = 2;

%e = error_ellipse([cov_mat(4),cov_mat(3);cov_mat(2),cov_mat(1)],...
%    'mu',[mean_vec(2), mean_vec(1)]);
%    e.Color = red_light;
%    e.LineWidth = 2;    

    hold off;

ax = gca;
ax.XTick = [];
ax.YTick = [];
xlabel(ax, 'y', 'FontName',font_name, 'FontSize',font_size, 'Interpreter','latex');
yl =ylabel(ax, 'x', 'FontName',font_name, 'FontSize',font_size, 'Interpreter','latex', 'Rotation',0);
yl.Position(1) = min(y)*0.999;

l = legend(ax, "\boldmath${z}_k$", "\boldmath${\mu}_z$", "\boldmath${P}_z$");
l.Interpreter = 'latex';
l.FontName = font_name;
l.FontSize = font_size;
l.Location = 'best';
l.Box = 'off';

%%
exportgraphics(f,'Zdroje/Obrazky/error_ellipse.png','Resolution',300)