function msg = nav_msgs_OccupancyGridStruct
% Message struct definition for nav_msgs/OccupancyGrid
coder.inline("never")
msg = struct(...
    'MessageType','nav_msgs/OccupancyGrid',...
    'header',std_msgs_HeaderStruct,...
    'info',nav_msgs_MapMetaDataStruct,...
    'data',ros.internal.ros2.messages.ros2.default_type('int8',NaN,1));
coder.cstructname(msg,'nav_msgs_OccupancyGridStruct_T');
coder.varsize('msg.data',[1000000000 1],[1 0]);
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
