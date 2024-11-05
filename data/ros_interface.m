t_max = 100;
t_s = 0.5;

data = zeros(round(t_max/t_s),5);
N = size(data,1);

rosinit;
state_sub = rossubscriber("/estimation/slam/state","DataFormat","struct");
cmd_sub = rossubscriber("/control/pure_pursuit/control_command","DataFormat","struct");
map_sub = rossubscriber("/estimation/slam/map", "DataFormat","struct");
pause(2);

map = receive(map_sub, 10);
disp("Map received");
for i = 1:N
    state = receive(state_sub,5);
    cmd = receive(cmd_sub,5);
    data(i,:) = [t_s*i,...
                    state.CarState_.X,...
                    state.CarState_.Y,...
                    state.CarStateDt.CarStateDt_.X,...
                    cmd.Throttle.Data];
    
    pause(t_s);
end
rosshutdown;


%% map_extract
blue_cones = zeros(length(map.ConeBlue),2);
yellow_cones = zeros(length(map.ConeYellow),2);

for i=1:length(map.ConeBlue)
    blue_cones(i,:) = [map.ConeBlue(i).Position.X map.ConeBlue(i).Position.Y];
end
for i=1:length(map.ConeYellow)
    yellow_cones(i,:) = [map.ConeYellow(i).Position.X map.ConeYellow(i).Position.Y];
end
%% plot
figure(1);
t1 = tiledlayout("flow");
t1.TileSpacing = 'compact';
t1.Padding = 'compact';

nexttile([2 1]);
plot(yellow_cones(:,2),yellow_cones(:,1),  "yo",LineWidth=1.5);
hold on;
plot(blue_cones(:,2), blue_cones(:,1), "bo",LineWidth=1.5)
plot(data(:,3), data(:,2), 'o', LineWidth=1);

nexttile;
plot(data(:,1), data(:,4));

nexttile;
plot(data(:,1), data(:,5));

