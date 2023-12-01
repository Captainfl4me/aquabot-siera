function slBusOut = RegionOfInterest(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.x_offset = uint32(msgIn.x_offset);
    slBusOut.y_offset = uint32(msgIn.y_offset);
    slBusOut.height = uint32(msgIn.height);
    slBusOut.width = uint32(msgIn.width);
    slBusOut.do_rectify = logical(msgIn.do_rectify);
end
