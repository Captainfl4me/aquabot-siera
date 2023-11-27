function rosmsgOut = OccupancyGrid(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.header = bus_conv_fcns.ros2.busToMsg.std_msgs.Header(slBusIn.header,rosmsgOut.header(1));
    rosmsgOut.info = bus_conv_fcns.ros2.busToMsg.nav_msgs.MapMetaData(slBusIn.info,rosmsgOut.info(1));
    rosmsgOut.data = int8(slBusIn.data(1:slBusIn.data_SL_Info.CurrentLength));
end
