function slBusOut = Int64(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    if (nargin == 5) && varargin{3} % Cast 64-bit integers to double
        slBusOut.data = double(msgIn.data);
    else
        slBusOut.data = int64(msgIn.data);
    end
end
