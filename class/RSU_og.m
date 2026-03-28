classdef RSU<handle
    properties(GetAccess=public)
        ID=[];
        
        % V2X_data
        v2x_data = struct();
        str_v2x_data = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
        str_v2x_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());

        % X2V_data
        x2v_data = struct();
        str_x2v_data = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
        str_x2v_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
    end
    properties
        X=0; Y=0; SHORT_K = 20; 
    end
    methods
        function obj=RSU(ID, X, Y)
            obj.ID=ID;
            obj.Y=Y;
            obj.X=X;
        end
    end

    methods(Static)
        % Recieve from vehicle to RSU
        function ReceivingV2X(obj, data)
            vehicleID = data.ID;
            fieldName = ['v', num2str(vehicleID)]; % Prepend 'v' to make a valid field name

            % Update single-value V2X data
            obj.v2x_data.(fieldName) = data;

            % append to direction logs (full + short)
            RSU.StoreV2XData(obj, data);
            RSU.StoreV2XDataShort(obj, data);
        end

        % Store V2X
        function StoreV2XData(obj, data)
            dir = upper(char(data.Dir));         % 'N','S','E','W'
            vfield = ['v', num2str(data.ID)];

            if ~isfield(obj.str_v2x_data.(dir), vfield)
                obj.str_v2x_data.(dir).(vfield) = initVehHistory();
            end
            H = obj.str_v2x_data.(dir).(vfield);

            % append
            H.t(end+1)  = data.t;
            H.X(end+1)  = data.X;
            H.Y(end+1)  = data.Y;
            H.V(end+1)  = data.V;
            H.Ac(end+1) = data.Ac;
            H.TRight(end+1)  = double(data.TurnRight);
            H.TRighted(end+1)= double(data.TurnedRight);
            H.TLeft(end+1)   = double(data.TurnLeft);
            H.TLefted(end+1) = double(data.TurnedLeft);

            obj.str_v2x_data.(dir).(vfield) = H;
        end

        % Store V2X Short
        function StoreV2XDataShort(obj, data)
            dir = upper(char(data.Dir));
            vfield = ['v', num2str(data.ID)];

            if ~isfield(obj.str_v2x_data_short.(dir), vfield)
                obj.str_v2x_data_short.(dir).(vfield) = initVehHistory();
            end
            Hs = obj.str_v2x_data_short.(dir).(vfield);

            % append
            Hs.t(end+1)  = data.t;
            Hs.X(end+1)  = data.X;
            Hs.Y(end+1)  = data.Y;
            Hs.V(end+1)  = data.V;
            Hs.Ac(end+1) = data.Ac;
            Hs.TRight(end+1)   = double(data.TurnRight);
            Hs.TRighted(end+1) = double(data.TurnedRight);
            Hs.TLeft(end+1)    = double(data.TurnLeft);
            Hs.TLefted(end+1)  = double(data.TurnedLeft);

            % keep only last SHORT_K
            K = obj.SHORT_K;
            Hs = trimLastK(Hs, K);

            obj.str_v2x_data_short.(dir).(vfield) = Hs;
        end

        % Reset V2X_short
        function Reset_Str_Short(obj, data)
            obj.str_v2x_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
        end

        % Calculating Arrival Time Prediction
        function ETA = arrival_time(obj)
            return;
        end

        % Calculating Free flow Arrival Time Prediction
        function FTA = free_flow_arrival_time(obj)
            return;
        end

        % Calculating Right-turn optimization
        function vpos = right_turn_opt(obj)
            return;
        end

        % vpos --> send --> if it is that car --> sending the turn_signal &
        % vpos

        % data = turn_signal + vpos
        % Sending from RSU back to vehicle
        function data = SendingX2V(obj)
            return;
        end

    end
end

% ------- helpers  -------
function H = initVehHistory()
H = struct('t',[], 'X',[], 'Y',[], 'V',[], 'Ac',[], ...
           'TRight',[], 'TRighted',[], 'TLeft',[], 'TLefted',[]);
end

function H = trimLastK(H, K)
fn = fieldnames(H);
for i = 1:numel(fn)
    v = H.(fn{i});
    if isnumeric(v)
        n = numel(v);
        if n > K, H.(fn{i}) = v(n-K+1:n); end
    end
end
end

