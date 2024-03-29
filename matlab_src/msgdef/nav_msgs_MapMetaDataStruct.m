function msg = nav_msgs_MapMetaDataStruct
% Message struct definition for nav_msgs/MapMetaData
coder.inline("never")
msg = struct(...
    'MessageType','nav_msgs/MapMetaData',...
    'map_load_time',builtin_interfaces_TimeStruct,...
    'resolution',ros.internal.ros2.messages.ros2.default_type('single',1,0),...
    'width',ros.internal.ros2.messages.ros2.default_type('uint32',1,0),...
    'height',ros.internal.ros2.messages.ros2.default_type('uint32',1,0),...
    'origin',geometry_msgs_PoseStruct);
coder.cstructname(msg,'nav_msgs_MapMetaDataStruct_T');
if ~isempty(coder.target)
    coder.ceval('//',coder.rref(msg));
end
end
