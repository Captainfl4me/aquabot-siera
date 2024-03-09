function slBusOut = ParamVec(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.header);
    for iter=1:currentlength
        slBusOut.header(iter) = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header(iter),slBusOut(1).header(iter),varargin{:});
    end
    slBusOut.header = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header,slBusOut(1).header,varargin{:});
    maxlength = length(slBusOut.params);
    recvdlength = length(msgIn.params);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'params', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.params_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.params_SL_Info.CurrentLength = uint32(currentlength);
    for iter=1:currentlength
        slBusOut.params(iter) = bus_conv_fcns.ros2.msgToBus.rcl_interfaces.Parameter(msgIn.params(iter),slBusOut(1).params(iter),varargin{:});
    end
end
