function rosmsgOut = MapMetaData(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.map_load_time = bus_conv_fcns.ros2.busToMsg.builtin_interfaces.Time(slBusIn.map_load_time,rosmsgOut.map_load_time(1));
    rosmsgOut.resolution = single(slBusIn.resolution);
    rosmsgOut.width = uint32(slBusIn.width);
    rosmsgOut.height = uint32(slBusIn.height);
    rosmsgOut.origin = bus_conv_fcns.ros2.busToMsg.geometry_msgs.Pose(slBusIn.origin,rosmsgOut.origin(1));
end
