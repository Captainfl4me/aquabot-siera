function slBusOut = ParameterValue(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.type = uint8(msgIn.type);
    slBusOut.bool_value = logical(msgIn.bool_value);
    if (nargin == 5) && varargin{3} % Cast 64-bit integers to double
        slBusOut.integer_value = double(msgIn.integer_value);
    else
        slBusOut.integer_value = int64(msgIn.integer_value);
    end
    slBusOut.double_value = double(msgIn.double_value);
    slBusOut.string_value_SL_Info.ReceivedLength = uint32(strlength(msgIn.string_value));
    currlen  = min(slBusOut.string_value_SL_Info.ReceivedLength, length(slBusOut.string_value));
    slBusOut.string_value_SL_Info.CurrentLength = uint32(currlen);
    slBusOut.string_value(1:currlen) = uint8(char(msgIn.string_value(1:currlen))).';
    maxlength = length(slBusOut.byte_array_value);
    recvdlength = length(msgIn.byte_array_value);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'byte_array_value', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.byte_array_value_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.byte_array_value_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.byte_array_value = uint8(msgIn.byte_array_value(1:slBusOut.byte_array_value_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.byte_array_value(recvdlength+1:maxlength) = 0;
    end
    maxlength = length(slBusOut.bool_array_value);
    recvdlength = length(msgIn.bool_array_value);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'bool_array_value', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.bool_array_value_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.bool_array_value_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.bool_array_value = logical(msgIn.bool_array_value(1:slBusOut.bool_array_value_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.bool_array_value(recvdlength+1:maxlength) = 0;
    end
    maxlength = length(slBusOut.integer_array_value);
    recvdlength = length(msgIn.integer_array_value);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'integer_array_value', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.integer_array_value_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.integer_array_value_SL_Info.CurrentLength = uint32(currentlength);
    if (nargin == 5) && varargin{3} % Cast 64-bit integers to double
        slBusOut.integer_array_value = double(msgIn.integer_array_value(1:slBusOut.integer_array_value_SL_Info.CurrentLength));
    else
        slBusOut.integer_array_value = int64(msgIn.integer_array_value(1:slBusOut.integer_array_value_SL_Info.CurrentLength));
    end
    if recvdlength < maxlength
    slBusOut.integer_array_value(recvdlength+1:maxlength) = 0;
    end
    maxlength = length(slBusOut.double_array_value);
    recvdlength = length(msgIn.double_array_value);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'double_array_value', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.double_array_value_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.double_array_value_SL_Info.CurrentLength = uint32(currentlength);
    slBusOut.double_array_value = double(msgIn.double_array_value(1:slBusOut.double_array_value_SL_Info.CurrentLength));
    if recvdlength < maxlength
    slBusOut.double_array_value(recvdlength+1:maxlength) = 0;
    end
    maxlength = length(slBusOut.string_array_value);
    recvdlength = length(msgIn.string_array_value);
    currentlength = min(maxlength, recvdlength);
    if (max(recvdlength) > maxlength) && ...
            isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
        diag = MSLDiagnostic([], ...
                             message('ros:slros:busconvert:TruncatedArray', ...
                                     'string_array_value', msgIn.MessageType, maxlength, max(recvdlength), maxlength, varargin{2}));
        reportAsWarning(diag);
    end
    slBusOut.string_array_value_SL_Info.ReceivedLength = uint32(recvdlength);
    slBusOut.string_array_value_SL_Info.CurrentLength = uint32(currentlength);
    for iter=1:currentlength
        recvlen = strlength(msgIn.string_array_value(iter));
        maxlen = length(slBusOut.string_array_value(iter).data);
        curlen = min(recvlen, maxlen);
        if (max(recvlen) > maxlen) && ...
                isequal(varargin{1}{1},ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning)
            diag = MSLDiagnostic([], ...
                                 message('ros:slros:busconvert:TruncatedArray', ...
                                         'string_array_value', msgIn.MessageType, maxlen, max(recvdlength), maxlength, varargin{2}));
            reportAsWarning(diag);
        end
        slBusOut.string_array_value(iter).data_SL_Info.CurrentLength = uint32(curlen);
        slBusOut.string_array_value(iter).data_SL_Info.ReceivedLength = uint32(recvlen);
        slBusOut.string_array_value(iter).data(1:curlen) = uint8(char(msgIn.string_array_value(iter)));
    end
end
