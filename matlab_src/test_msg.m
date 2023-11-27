
myNode = ros2node("/MW_node");
sub = ros2subscriber(myNode,'/wamv/sensors/acoustics/receiver/range_bearing');
msg = receive(sub);

msg.params