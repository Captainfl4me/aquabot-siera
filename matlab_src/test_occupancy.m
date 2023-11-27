close all
clear
clc

ros_node = ros2node("test_node");
sub = ros2subscriber(ros_node, "/wamv/sensors/lidars/lidar_wamv_sensor/scan");
scan_msg = receive(sub);

ranges = scan_msg.ranges;
angles = transpose(linspace(scan_msg.angle_min, scan_msg.angle_max, length(ranges)));

ox = ranges.*cos(angles);
oy = ranges.*sin(angles);

ox_corr = ox(isfinite(ox));
oy_corr = oy(isfinite(oy));

figure
scatter(ox_corr, oy_corr);

map = binaryOccupancyMap(260,260,1);
map.GridOriginInLocal = [-130 -130];
if length(ox_corr) > 0
    map.setOccupancy([ox_corr oy_corr], 1);
    map.inflate(1);
end
figure
show(map)

msg = ros2message("nav_msgs/OccupancyGrid")
matrix = map.occupancyMatrix();
msg.data = matrix(:)
