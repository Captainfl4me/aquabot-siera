function slBusOut = Parameter(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.name_SL_Info.ReceivedLength = uint32(strlength(msgIn.name));
    currlen  = min(slBusOut.name_SL_Info.ReceivedLength, length(slBusOut.name));
    slBusOut.name_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.name(1:currlen) = uint8(char(msgIn.name(1:currlen))).';
    currentlength = length(slBusOut.value);
    for iter=1:currentlength
        slBusOut.value(iter) = bus_conv_fcns.ros2.msgToBus.rcl_interfaces.ParameterValue(msgIn.value(iter),slBusOut(1).value(iter),varargin{:});
    end
    slBusOut.value = bus_conv_fcns.ros2.msgToBus.rcl_interfaces.ParameterValue(msgIn.value,slBusOut(1).value,varargin{:});
end
