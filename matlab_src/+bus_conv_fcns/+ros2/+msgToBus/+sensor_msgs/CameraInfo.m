function slBusOut = CameraInfo(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    currentlength = length(slBusOut.header);
    for iter=1:currentlength
        slBusOut.header(iter) = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header(iter),slBusOut(1).header(iter),varargin{:});
    end
    slBusOut.header = bus_conv_fcns.ros2.msgToBus.std_msgs.Header(msgIn.header,slBusOut(1).header,varargin{:});
    slBusOut.height = uint32(msgIn.height);
    slBusOut.width = uint32(msgIn.width);
    slBusOut.distortion_model_SL_Info.ReceivedLength = uint32(strlength(msgIn.distortion_model));
    currlen  = min(slBusOut.distortion_model_SL_Info.ReceivedLength, length(slBusOut.distortion_model));
    slBusOut.distortion_model_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.distortion_model(1:currlen) = uint8(char(msgIn.distortion_model(1:currlen))).';
    maxlength = length(slBusOut.d);
    recvdlength = length(msgIn.d);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'd', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.d_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.d_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.d = double(msgIn.d(1:slBusOut.d_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.d(recvdlength+1:maxlength) = 0;
    end
                    currentlength = length(slBusOut.k);
                    slBusOut.k = double(msgIn.k(1:currentlength));
                    currentlength = length(slBusOut.r);
                    slBusOut.r = double(msgIn.r(1:currentlength));
                    currentlength = length(slBusOut.p);
                    slBusOut.p = double(msgIn.p(1:currentlength));
    slBusOut.binning_x = uint32(msgIn.binning_x);
    slBusOut.binning_y = uint32(msgIn.binning_y);
    currentlength = length(slBusOut.roi);
    for iter=1:currentlength
        slBusOut.roi(iter) = bus_conv_fcns.ros2.msgToBus.sensor_msgs.RegionOfInterest(msgIn.roi(iter),slBusOut(1).roi(iter),varargin{:});
    end
    slBusOut.roi = bus_conv_fcns.ros2.msgToBus.sensor_msgs.RegionOfInterest(msgIn.roi,slBusOut(1).roi,varargin{:});
end
