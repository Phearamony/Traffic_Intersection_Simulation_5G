classdef RSU<handle
    properties(GetAccess=public)
        ID=[];
        
        % V2X_data (received from vehicles)
        v2x_data = struct();          % Latest data per vehicle (quick access)
        str_v2x_data = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());        % Permanent
        str_v2x_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());  % Temp

        % X2V_data (sent to vehicles)
        x2v_data = struct();          % Latest sent data per vehicle
        str_x2v_data = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());        % Permanent
        str_x2v_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());  % Temp
        
        % Track which vehicles are active
        active_vehicles = struct('N',[],'S',[],'E',[],'W',[]);
        
        % ============================================================
        % [ADDED] Optimization results storage
        % ============================================================
        opt_results = struct();  % Store latest optimization results
        coordinated_times = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
    end

    properties
        X=0; Y=0; 
        SHORT_K = 20;              % keep last K entries in short storage
        COMM_RANGE = 200;          % communication range in meters
        INTERSECTION_RADIUS = 15;  % intersection zone radius
        
        % ============================================================
        % [ADDED] Optimization parameters from paper
        % ============================================================
        tau_safe = 4.0;    % [s] Safe time gap for turning maneuver
        h_time = 2.0;      % [s] Minimum car-following headway
        omega_East = 1.0;  % Weight for East direction delay
        omega_West = 1.0;  % Weight for West direction delay
        omega_NS = 1.0;    % Weight for NS direction delay
        Vd = 25;           % [m/s] Desired velocity for estimation
        dt_sim = 0.1;      % [s] Simulation timestep for arrival estimation
        stop_line = 10;    % [m] Distance from center to stop line
    end

    methods
        function obj=RSU(ID, X, Y)
            obj.ID=ID;
            obj.Y=Y;
            obj.X=X;
        end
    end

    methods(Static)
        %% Initialize vehicle history structure
        function H = initVehHistory()
            H = struct('t',[], 'X',[], 'Y',[], 'V',[], 'Ac',[], ...
                       'TRight',[], 'TRighted',[], 'TLeft',[], 'TLefted',[]);
        end

        %% Initialize X2V history structure
        function H = initX2VHistory()
            H = struct('t',[], 'turn_signal',[], 'recommended_V',[], ...
                       'recommended_Ac',[], 'priority',[], 'message',[], ...
                       'target_arrival_time',[]);
        end

        %% Trim to last K entries
        function H = trimLastK(H, K)
            fn = fieldnames(H);
            for i = 1:numel(fn)
                v = H.(fn{i});
                if isnumeric(v)
                    n = numel(v);
                    if n > K
                        H.(fn{i}) = v(n-K+1:n);
                    end
                elseif iscell(v)
                    n = numel(v);
                    if n > K
                        H.(fn{i}) = v(n-K+1:n);
                    end
                end
            end
        end
    end

    methods
        %% ==================== V2X RECEIVING ====================
        
        %% Receive data from vehicle to RSU
        function ReceivingV2X(obj, data)
            vehicleID = data.ID;
            vfield = ['v', num2str(vehicleID)];
            dir = upper(char(data.Dir));

            % Store latest V2X data per vehicle
            obj.v2x_data.(vfield) = data;

            % Track active vehicle
            if ~ismember(vehicleID, obj.active_vehicles.(dir))
                obj.active_vehicles.(dir)(end+1) = vehicleID;
            end

            % Store in permanent history
            obj.StoreV2XData(data);
            
            % Store in temp (short) history
            obj.StoreV2XDataShort(data);
        end

        %% Store V2X data (permanent)
        function StoreV2XData(obj, data)
            dir = upper(char(data.Dir));
            vfield = ['v', num2str(data.ID)];

            % Initialize if doesn't exist
            if ~isfield(obj.str_v2x_data.(dir), vfield)
                obj.str_v2x_data.(dir).(vfield) = RSU.initVehHistory();
            end
            H = obj.str_v2x_data.(dir).(vfield);

            % Append data
            H.t(end+1) = data.t;
            H.X(end+1) = data.X;
            H.Y(end+1) = data.Y;
            H.V(end+1) = data.V;
            H.Ac(end+1) = data.Ac;
            H.TRight(end+1) = double(data.TurnRight);
            H.TRighted(end+1) = double(data.TurnedRight);
            H.TLeft(end+1) = double(data.TurnLeft);
            H.TLefted(end+1) = double(data.TurnedLeft);

            obj.str_v2x_data.(dir).(vfield) = H;
        end

        %% Store V2X data (temp/short)
        function StoreV2XDataShort(obj, data)
            dir = upper(char(data.Dir));
            vfield = ['v', num2str(data.ID)];

            % Initialize if doesn't exist
            if ~isfield(obj.str_v2x_data_short.(dir), vfield)
                obj.str_v2x_data_short.(dir).(vfield) = RSU.initVehHistory();
            end
            Hs = obj.str_v2x_data_short.(dir).(vfield);

            % Append data
            Hs.t(end+1) = data.t;
            Hs.X(end+1) = data.X;
            Hs.Y(end+1) = data.Y;
            Hs.V(end+1) = data.V;
            Hs.Ac(end+1) = data.Ac;
            Hs.TRight(end+1) = double(data.TurnRight);
            Hs.TRighted(end+1) = double(data.TurnedRight);
            Hs.TLeft(end+1) = double(data.TurnLeft);
            Hs.TLefted(end+1) = double(data.TurnedLeft);

            % Trim to last K
            Hs = RSU.trimLastK(Hs, obj.SHORT_K);

            obj.str_v2x_data_short.(dir).(vfield) = Hs;
        end

        %% Reset V2X short storage
        function ResetV2XShort(obj)
            obj.str_v2x_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
        end

        %% Reset V2X short storage for specific direction
        function ResetV2XShortDir(obj, dir)
            obj.str_v2x_data_short.(upper(dir)) = struct();
        end

        %% ==================== X2V SENDING ====================
        
        %% Send data from RSU to specific vehicle
        function data = SendingX2V(obj, t, vehicleID, turn_signal, recommended_V, recommended_Ac, priority, message, target_arrival_time)
            if nargin < 9
                target_arrival_time = NaN;
            end
            
            vfield = ['v', num2str(vehicleID)];
            
            % Create data packet
            data = struct(...
                't', t, ...
                'target_ID', vehicleID, ...
                'turn_signal', turn_signal, ...
                'recommended_V', recommended_V, ...
                'recommended_Ac', recommended_Ac, ...
                'priority', priority, ...
                'message', message, ...
                'target_arrival_time', target_arrival_time ...
            );
            
            % Store latest X2V data per vehicle
            obj.x2v_data.(vfield) = data;
            
            % Get direction from v2x_data if available
            if isfield(obj.v2x_data, vfield) && isfield(obj.v2x_data.(vfield), 'Dir')
                dir = upper(char(obj.v2x_data.(vfield).Dir));
                
                % Store in permanent history
                obj.StoreX2VData(dir, vehicleID, data);
                
                % Store in temp (short) history
                obj.StoreX2VDataShort(dir, vehicleID, data);
            end
        end

        %% Store X2V data (permanent)
        function StoreX2VData(obj, dir, vehicleID, data)
            vfield = ['v', num2str(vehicleID)];

            % Initialize if doesn't exist
            if ~isfield(obj.str_x2v_data.(dir), vfield)
                obj.str_x2v_data.(dir).(vfield) = RSU.initX2VHistory();
            end
            H = obj.str_x2v_data.(dir).(vfield);

            % Append data
            H.t(end+1) = data.t;
            H.turn_signal(end+1) = data.turn_signal;
            H.recommended_V(end+1) = data.recommended_V;
            H.recommended_Ac(end+1) = data.recommended_Ac;
            H.priority(end+1) = data.priority;
            H.message{end+1} = data.message;
            H.target_arrival_time(end+1) = data.target_arrival_time;

            obj.str_x2v_data.(dir).(vfield) = H;
        end

        %% Store X2V data (temp/short)
        function StoreX2VDataShort(obj, dir, vehicleID, data)
            vfield = ['v', num2str(vehicleID)];

            % Initialize if doesn't exist
            if ~isfield(obj.str_x2v_data_short.(dir), vfield)
                obj.str_x2v_data_short.(dir).(vfield) = RSU.initX2VHistory();
            end
            Hs = obj.str_x2v_data_short.(dir).(vfield);

            % Append data
            Hs.t(end+1) = data.t;
            Hs.turn_signal(end+1) = data.turn_signal;
            Hs.recommended_V(end+1) = data.recommended_V;
            Hs.recommended_Ac(end+1) = data.recommended_Ac;
            Hs.priority(end+1) = data.priority;
            Hs.message{end+1} = data.message;
            Hs.target_arrival_time(end+1) = data.target_arrival_time;

            % Trim to last K
            Hs = RSU.trimLastK(Hs, obj.SHORT_K);

            obj.str_x2v_data_short.(dir).(vfield) = Hs;
        end

        %% Reset X2V short storage
        function ResetX2VShort(obj)
            obj.str_x2v_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
        end

        %% Reset all short storage
        function ResetAllShort(obj)
            obj.ResetV2XShort();
            obj.ResetX2VShort();
        end

        %% ==================== VEHICLE MANAGEMENT ====================
        
        %% Remove vehicle from tracking
        function RemoveVehicle(obj, vehicleID, dir)
            dir = upper(char(dir));
            vfield = ['v', num2str(vehicleID)];
            
            obj.active_vehicles.(dir)(obj.active_vehicles.(dir) == vehicleID) = [];
            
            if isfield(obj.str_v2x_data_short.(dir), vfield)
                obj.str_v2x_data_short.(dir) = rmfield(obj.str_v2x_data_short.(dir), vfield);
            end
            if isfield(obj.str_x2v_data_short.(dir), vfield)
                obj.str_x2v_data_short.(dir) = rmfield(obj.str_x2v_data_short.(dir), vfield);
            end
        end

        %% Get all active vehicles in a direction
        function ids = GetActiveVehicles(obj, dir)
            ids = obj.active_vehicles.(upper(dir));
        end

        %% Get latest V2X data for a vehicle
        function data = GetVehicleData(obj, vehicleID)
            vfield = ['v', num2str(vehicleID)];
            if isfield(obj.v2x_data, vfield)
                data = obj.v2x_data.(vfield);
            else
                data = [];
            end
        end

        %% ==================== ARRIVAL TIME ESTIMATION ====================
        %% (Based on paper's iterative forward simulation)
        
        %% Estimate arrival time for a single vehicle using forward simulation
        function [tau, trajectory] = EstimateArrivalTime(obj, vehicleID)
            data = obj.GetVehicleData(vehicleID);
            if isempty(data)
                tau = Inf;
                trajectory = [];
                return;
            end
            
            % Get current state
            switch data.Dir
                case 'N'
                    pos = data.Y;
                    target = -obj.stop_line;  % Stop line position
                    direction = 1;  % Moving in positive Y
                case 'S'
                    pos = data.Y;
                    target = obj.stop_line;
                    direction = -1;  % Moving in negative Y
                case 'E'
                    pos = data.X;
                    target = -obj.stop_line;
                    direction = 1;
                case 'W'
                    pos = data.X;
                    target = obj.stop_line;
                    direction = -1;
                otherwise
                    tau = Inf;
                    trajectory = [];
                    return;
            end
            
            % Check if already past stop line
            if direction == 1 && pos >= target
                tau = 0;
                trajectory = struct('t', 0, 'pos', pos, 'vel', data.V);
                return;
            elseif direction == -1 && pos <= target
                tau = 0;
                trajectory = struct('t', 0, 'pos', pos, 'vel', data.V);
                return;
            end
            
            % Forward simulation using IDM-like behavior
            v = data.V;
            p = pos;
            t = 0;
            max_time = 60;  % Maximum simulation time
            
            trajectory = struct('t', [], 'pos', [], 'vel', []);
            
            while t < max_time
                % Store trajectory
                trajectory.t(end+1) = t;
                trajectory.pos(end+1) = p;
                trajectory.vel(end+1) = v;
                
                % Check if reached target
                if (direction == 1 && p >= target) || (direction == -1 && p <= target)
                    tau = t;
                    return;
                end
                
                % IDM acceleration toward desired velocity (free flow)
                a_max = 1.5;
                a = a_max * (1 - (v / obj.Vd)^4);
                a = max(min(a, 2), -7);  % Clamp acceleration
                
                % Update state
                p = p + direction * v * obj.dt_sim + 0.5 * direction * a * obj.dt_sim^2;
                v = v + a * obj.dt_sim;
                v = max(v, 0);  % No negative velocity
                t = t + obj.dt_sim;
            end
            
            tau = Inf;  % Did not reach in time
        end
        
        %% Estimate arrival times for all vehicles in a direction
        function [ids, taus] = EstimateAllArrivalTimes(obj, dir)
            ids = obj.GetActiveVehicles(dir);
            n = length(ids);
            taus = zeros(1, n);
            
            for i = 1:n
                taus(i) = obj.EstimateArrivalTime(ids(i));
            end
            
            % Sort by arrival time
            [taus, sortIdx] = sort(taus);
            ids = ids(sortIdx);
        end
        
        %% ==================== RIGHT-TURN OPTIMIZATION ====================
        
        %% Main optimization function for EW conflict (W turning right, E going straight)
        function [opt_p, coordinated_tau_E, coordinated_tau_W, min_cost] = ...
                OptimizeRightTurn_EW(obj, turningCarID)
            %
            % Optimizes right-turn timing for a WEST car turning into NORTH
            % against EAST cars going straight (or turning)
            %
            % Based on paper formulation:
            %   J = omega_E * sum((tau_bar_e - tau_e)^2) + omega_W * sum((tau_bar_w - tau_w)^2)
            %
            % Returns:
            %   opt_p: optimal gap position (turn after East vehicle p-1)
            %   coordinated_tau_E: coordinated arrival times for East vehicles
            %   coordinated_tau_W: coordinated arrival times for West vehicles
            %   min_cost: minimum total delay cost
            
            % Get turning vehicle data
            turningData = obj.GetVehicleData(turningCarID);
            if isempty(turningData) || ~turningData.TurnRight
                opt_p = 0;
                coordinated_tau_E = [];
                coordinated_tau_W = [];
                min_cost = Inf;
                return;
            end
            
            % Get all vehicles and their natural arrival times
            [ids_E, tau_bar_E] = obj.EstimateAllArrivalTimes('E');
            [ids_W, tau_bar_W] = obj.EstimateAllArrivalTimes('W');
            
            M = length(ids_E);  % Number of East vehicles
            N = length(ids_W);  % Number of West vehicles
            
            % Find position q of turning vehicle in West queue
            q = find(ids_W == turningCarID);
            if isempty(q)
                opt_p = 0;
                coordinated_tau_E = tau_bar_E;
                coordinated_tau_W = tau_bar_W;
                min_cost = Inf;
                return;
            end
            
            % Filter East vehicles: only those going straight (not turning right)
            straight_E_mask = true(1, M);
            for i = 1:M
                eData = obj.GetVehicleData(ids_E(i));
                if ~isempty(eData) && eData.TurnRight
                    straight_E_mask(i) = false;
                end
            end
            
            % Natural arrival time of turning vehicle
            tau_bar_wq = tau_bar_W(q);
            
            % Try each possible gap position p = 1, 2, ..., M+1
            % p = 1: turn before all East vehicles
            % p = k: turn after East vehicle k-1, before vehicle k
            % p = M+1: turn after all East vehicles
            
            min_cost = Inf;
            opt_p = M + 1;  % Default: wait for all
            best_tau_E = tau_bar_E;
            best_tau_W = tau_bar_W;
            
            for p = 1:(M + 1)
                % Calculate coordinated times for this gap position
                [tau_E, tau_W, feasible] = obj.ComputeCoordinatedTimes_EW(...
                    tau_bar_E, tau_bar_W, p, q, straight_E_mask);
                
                if ~feasible
                    continue;
                end
                
                % Calculate cost: sum of squared delays
                delay_E = tau_E - tau_bar_E;
                delay_W = tau_W - tau_bar_W;
                
                cost = obj.omega_East * sum(delay_E.^2) + ...
                       obj.omega_West * sum(delay_W.^2);
                
                if cost < min_cost
                    min_cost = cost;
                    opt_p = p;
                    best_tau_E = tau_E;
                    best_tau_W = tau_W;
                end
            end
            
            coordinated_tau_E = best_tau_E;
            coordinated_tau_W = best_tau_W;
            
            % Store results
            obj.opt_results.EW = struct(...
                'opt_p', opt_p, ...
                'tau_E', coordinated_tau_E, ...
                'tau_W', coordinated_tau_W, ...
                'ids_E', ids_E, ...
                'ids_W', ids_W, ...
                'turningCarID', turningCarID, ...
                'min_cost', min_cost ...
            );
        end
        
        %% Compute coordinated times for a given gap position (EW case)
        function [tau_E, tau_W, feasible] = ComputeCoordinatedTimes_EW(...
                obj, tau_bar_E, tau_bar_W, p, q, straight_E_mask)
            %
            % Given gap position p and turning vehicle position q,
            % compute the coordinated arrival times respecting constraints
            %
            
            M = length(tau_bar_E);
            N = length(tau_bar_W);
            
            tau_E = tau_bar_E;  % Start with natural times
            tau_W = tau_bar_W;
            
            feasible = true;
            
            % Constraints:
            % 1. Vehicles 1 to p-1 in East keep natural times (unaffected)
            % 2. Vehicles 1 to q-1 in West keep natural times (unaffected)
            % 3. tau_wq >= tau_e(p-1) + tau_safe  (turning after East p-1)
            % 4. tau_ep >= tau_wq + tau_safe      (East p waits for turn)
            % 5. Car-following: tau_em >= tau_e(m-1) + h_time for m > p
            % 6. Car-following: tau_wn >= tau_w(n-1) + h_time for n > q
            % 7. No negative delays: tau >= tau_bar
            
            % Step 1: Turning vehicle timing
            if p == 1
                % Turn before all East vehicles
                tau_wq = tau_bar_W(q);  % Can go at natural time
            else
                % Turn after East vehicle p-1
                tau_e_prev = tau_E(p-1);
                tau_wq = max(tau_bar_W(q), tau_e_prev + obj.tau_safe);
            end
            tau_W(q) = tau_wq;
            
            % Step 2: East vehicle p must wait for turn to complete
            if p <= M && straight_E_mask(p)
                tau_E(p) = max(tau_bar_E(p), tau_wq + obj.tau_safe);
            end
            
            % Step 3: Propagate car-following for East vehicles after p
            for m = (p+1):M
                if straight_E_mask(m)
                    tau_E(m) = max(tau_bar_E(m), tau_E(m-1) + obj.h_time);
                end
            end
            
            % Step 4: Propagate car-following for West vehicles after q
            for n = (q+1):N
                tau_W(n) = max(tau_bar_W(n), tau_W(n-1) + obj.h_time);
            end
            
            % Check feasibility: no negative delays allowed
            if any(tau_E < tau_bar_E - 0.01) || any(tau_W < tau_bar_W - 0.01)
                feasible = false;
            end
        end
        
        %% Main optimization function for NS conflict (N turning right, S going straight)
        function [opt_p, coordinated_tau_S, coordinated_tau_N, min_cost] = ...
                OptimizeRightTurn_NS(obj, turningCarID)
            %
            % Optimizes right-turn timing for a NORTH car turning into EAST
            % against SOUTH cars going straight
            %
            
            turningData = obj.GetVehicleData(turningCarID);
            if isempty(turningData) || ~turningData.TurnRight
                opt_p = 0;
                coordinated_tau_S = [];
                coordinated_tau_N = [];
                min_cost = Inf;
                return;
            end
            
            [ids_S, tau_bar_S] = obj.EstimateAllArrivalTimes('S');
            [ids_N, tau_bar_N] = obj.EstimateAllArrivalTimes('N');
            
            M = length(ids_S);  % Oncoming vehicles
            N = length(ids_N);  % Same direction as turner
            
            q = find(ids_N == turningCarID);
            if isempty(q)
                opt_p = 0;
                coordinated_tau_S = tau_bar_S;
                coordinated_tau_N = tau_bar_N;
                min_cost = Inf;
                return;
            end
            
            % Filter: only straight-going South vehicles
            straight_S_mask = true(1, M);
            for i = 1:M
                sData = obj.GetVehicleData(ids_S(i));
                if ~isempty(sData) && sData.TurnRight
                    straight_S_mask(i) = false;
                end
            end
            
            min_cost = Inf;
            opt_p = M + 1;
            best_tau_S = tau_bar_S;
            best_tau_N = tau_bar_N;
            
            for p = 1:(M + 1)
                [tau_S, tau_N, feasible] = obj.ComputeCoordinatedTimes_NS(...
                    tau_bar_S, tau_bar_N, p, q, straight_S_mask);
                
                if ~feasible
                    continue;
                end
                
                delay_S = tau_S - tau_bar_S;
                delay_N = tau_N - tau_bar_N;
                
                cost = obj.omega_NS * sum(delay_S.^2) + ...
                       obj.omega_NS * sum(delay_N.^2);
                
                if cost < min_cost
                    min_cost = cost;
                    opt_p = p;
                    best_tau_S = tau_S;
                    best_tau_N = tau_N;
                end
            end
            
            coordinated_tau_S = best_tau_S;
            coordinated_tau_N = best_tau_N;
            
            obj.opt_results.NS = struct(...
                'opt_p', opt_p, ...
                'tau_S', coordinated_tau_S, ...
                'tau_N', coordinated_tau_N, ...
                'ids_S', ids_S, ...
                'ids_N', ids_N, ...
                'turningCarID', turningCarID, ...
                'min_cost', min_cost ...
            );
        end
        
        %% Compute coordinated times for NS case
        function [tau_S, tau_N, feasible] = ComputeCoordinatedTimes_NS(...
                obj, tau_bar_S, tau_bar_N, p, q, straight_S_mask)
            
            M = length(tau_bar_S);
            N = length(tau_bar_N);
            
            tau_S = tau_bar_S;
            tau_N = tau_bar_N;
            feasible = true;
            
            % Turning vehicle timing
            if p == 1
                tau_nq = tau_bar_N(q);
            else
                tau_s_prev = tau_S(p-1);
                tau_nq = max(tau_bar_N(q), tau_s_prev + obj.tau_safe);
            end
            tau_N(q) = tau_nq;
            
            % South vehicle p waits
            if p <= M && straight_S_mask(p)
                tau_S(p) = max(tau_bar_S(p), tau_nq + obj.tau_safe);
            end
            
            % Propagate car-following
            for m = (p+1):M
                if straight_S_mask(m)
                    tau_S(m) = max(tau_bar_S(m), tau_S(m-1) + obj.h_time);
                end
            end
            
            for n = (q+1):N
                tau_N(n) = max(tau_bar_N(n), tau_N(n-1) + obj.h_time);
            end
            
            if any(tau_S < tau_bar_S - 0.01) || any(tau_N < tau_bar_N - 0.01)
                feasible = false;
            end
        end
        
        %% ==================== VELOCITY RECOMMENDATION ====================
        
        %% Calculate recommended velocity to meet target arrival time
        function V_rec = CalculateRecommendedVelocity(obj, vehicleID, target_tau, current_t)
            data = obj.GetVehicleData(vehicleID);
            if isempty(data)
                V_rec = NaN;
                return;
            end
            
            % Calculate distance to stop line
            switch data.Dir
                case 'N'
                    dist = abs(-obj.stop_line - data.Y);
                case 'S'
                    dist = abs(obj.stop_line - data.Y);
                case 'E'
                    dist = abs(-obj.stop_line - data.X);
                case 'W'
                    dist = abs(obj.stop_line - data.X);
                otherwise
                    V_rec = NaN;
                    return;
            end
            
            % Time available
            time_available = target_tau - current_t;
            
            if time_available <= 0
                V_rec = data.V;  % Keep current speed
                return;
            end
            
            % Simple: V = distance / time
            V_rec = dist / time_available;
            
            % Clamp to reasonable range
            V_rec = max(min(V_rec, obj.Vd * 1.2), 0);
        end
        
        %% ==================== HIGH-LEVEL COORDINATION ====================
        
        %% Run optimization at start of green phase
        function RunGreenPhaseOptimization(obj, t, greenDir)
            %
            % Call this at the start of each green phase
            % greenDir: 'NS' or 'EW'
            %
            
            if strcmp(greenDir, 'EW')
                % Check for right-turning W vehicles
                ids_W = obj.GetActiveVehicles('W');
                for i = 1:length(ids_W)
                    data = obj.GetVehicleData(ids_W(i));
                    if ~isempty(data) && data.TurnRight && ~data.TurnedRight
                        % Found a right-turning vehicle, run optimization
                        obj.OptimizeRightTurn_EW(ids_W(i));
                        break;  % Only optimize for first turner (paper assumption)
                    end
                end
                
                % Check for right-turning E vehicles
                ids_E = obj.GetActiveVehicles('E');
                for i = 1:length(ids_E)
                    data = obj.GetVehicleData(ids_E(i));
                    if ~isempty(data) && data.TurnRight && ~data.TurnedRight
                        % E turning right conflicts with W going straight
                        obj.OptimizeRightTurn_EW_Reverse(ids_E(i));
                        break;
                    end
                end
                
            elseif strcmp(greenDir, 'NS')
                % Check for right-turning N vehicles
                ids_N = obj.GetActiveVehicles('N');
                for i = 1:length(ids_N)
                    data = obj.GetVehicleData(ids_N(i));
                    if ~isempty(data) && data.TurnRight && ~data.TurnedRight
                        obj.OptimizeRightTurn_NS(ids_N(i));
                        break;
                    end
                end
                
                % Check for right-turning S vehicles
                ids_S = obj.GetActiveVehicles('S');
                for i = 1:length(ids_S)
                    data = obj.GetVehicleData(ids_S(i));
                    if ~isempty(data) && data.TurnRight && ~data.TurnedRight
                        obj.OptimizeRightTurn_NS_Reverse(ids_S(i));
                        break;
                    end
                end
            end
        end
        
        %% E turning right (reverse of W turning)
        function [opt_p, coordinated_tau_W, coordinated_tau_E, min_cost] = ...
                OptimizeRightTurn_EW_Reverse(obj, turningCarID)
            % E car turning right, W cars going straight
            
            turningData = obj.GetVehicleData(turningCarID);
            if isempty(turningData) || ~turningData.TurnRight
                opt_p = 0;
                coordinated_tau_W = [];
                coordinated_tau_E = [];
                min_cost = Inf;
                return;
            end
            
            [ids_W, tau_bar_W] = obj.EstimateAllArrivalTimes('W');
            [ids_E, tau_bar_E] = obj.EstimateAllArrivalTimes('E');
            
            M = length(ids_W);
            N = length(ids_E);
            
            q = find(ids_E == turningCarID);
            if isempty(q)
                opt_p = 0;
                coordinated_tau_W = tau_bar_W;
                coordinated_tau_E = tau_bar_E;
                min_cost = Inf;
                return;
            end
            
            straight_W_mask = true(1, M);
            for i = 1:M
                wData = obj.GetVehicleData(ids_W(i));
                if ~isempty(wData) && wData.TurnRight
                    straight_W_mask(i) = false;
                end
            end
            
            min_cost = Inf;
            opt_p = M + 1;
            best_tau_W = tau_bar_W;
            best_tau_E = tau_bar_E;
            
            for p = 1:(M + 1)
                tau_W = tau_bar_W;
                tau_E = tau_bar_E;
                feasible = true;
                
                if p == 1
                    tau_eq = tau_bar_E(q);
                else
                    tau_eq = max(tau_bar_E(q), tau_W(p-1) + obj.tau_safe);
                end
                tau_E(q) = tau_eq;
                
                if p <= M && straight_W_mask(p)
                    tau_W(p) = max(tau_bar_W(p), tau_eq + obj.tau_safe);
                end
                
                for m = (p+1):M
                    if straight_W_mask(m)
                        tau_W(m) = max(tau_bar_W(m), tau_W(m-1) + obj.h_time);
                    end
                end
                
                for n = (q+1):N
                    tau_E(n) = max(tau_bar_E(n), tau_E(n-1) + obj.h_time);
                end
                
                if any(tau_W < tau_bar_W - 0.01) || any(tau_E < tau_bar_E - 0.01)
                    feasible = false;
                end
                
                if ~feasible
                    continue;
                end
                
                delay_W = tau_W - tau_bar_W;
                delay_E = tau_E - tau_bar_E;
                cost = obj.omega_West * sum(delay_W.^2) + obj.omega_East * sum(delay_E.^2);
                
                if cost < min_cost
                    min_cost = cost;
                    opt_p = p;
                    best_tau_W = tau_W;
                    best_tau_E = tau_E;
                end
            end
            
            coordinated_tau_W = best_tau_W;
            coordinated_tau_E = best_tau_E;
        end
        
        %% S turning right (reverse of N turning)
        function [opt_p, coordinated_tau_N, coordinated_tau_S, min_cost] = ...
                OptimizeRightTurn_NS_Reverse(obj, turningCarID)
            % S car turning right, N cars going straight
            
            turningData = obj.GetVehicleData(turningCarID);
            if isempty(turningData) || ~turningData.TurnRight
                opt_p = 0;
                coordinated_tau_N = [];
                coordinated_tau_S = [];
                min_cost = Inf;
                return;
            end
            
            [ids_N, tau_bar_N] = obj.EstimateAllArrivalTimes('N');
            [ids_S, tau_bar_S] = obj.EstimateAllArrivalTimes('S');
            
            M = length(ids_N);
            N = length(ids_S);
            
            q = find(ids_S == turningCarID);
            if isempty(q)
                opt_p = 0;
                coordinated_tau_N = tau_bar_N;
                coordinated_tau_S = tau_bar_S;
                min_cost = Inf;
                return;
            end
            
            straight_N_mask = true(1, M);
            for i = 1:M
                nData = obj.GetVehicleData(ids_N(i));
                if ~isempty(nData) && nData.TurnRight
                    straight_N_mask(i) = false;
                end
            end
            
            min_cost = Inf;
            opt_p = M + 1;
            best_tau_N = tau_bar_N;
            best_tau_S = tau_bar_S;
            
            for p = 1:(M + 1)
                tau_N = tau_bar_N;
                tau_S = tau_bar_S;
                feasible = true;
                
                if p == 1
                    tau_sq = tau_bar_S(q);
                else
                    tau_sq = max(tau_bar_S(q), tau_N(p-1) + obj.tau_safe);
                end
                tau_S(q) = tau_sq;
                
                if p <= M && straight_N_mask(p)
                    tau_N(p) = max(tau_bar_N(p), tau_sq + obj.tau_safe);
                end
                
                for m = (p+1):M
                    if straight_N_mask(m)
                        tau_N(m) = max(tau_bar_N(m), tau_N(m-1) + obj.h_time);
                    end
                end
                
                for n = (q+1):N
                    tau_S(n) = max(tau_bar_S(n), tau_S(n-1) + obj.h_time);
                end
                
                if any(tau_N < tau_bar_N - 0.01) || any(tau_S < tau_bar_S - 0.01)
                    feasible = false;
                end
                
                if ~feasible
                    continue;
                end
                
                delay_N = tau_N - tau_bar_N;
                delay_S = tau_S - tau_bar_S;
                cost = obj.omega_NS * sum(delay_N.^2) + obj.omega_NS * sum(delay_S.^2);
                
                if cost < min_cost
                    min_cost = cost;
                    opt_p = p;
                    best_tau_N = tau_N;
                    best_tau_S = tau_S;
                end
            end
            
            coordinated_tau_N = best_tau_N;
            coordinated_tau_S = best_tau_S;
        end
        
        %% Get coordinated arrival time for a specific vehicle
        function tau = GetCoordinatedArrivalTime(obj, vehicleID, dir)
            vfield = ['v', num2str(vehicleID)];
            tau = NaN;
            
            % Check EW optimization results
            if isfield(obj.opt_results, 'EW') && ~isempty(obj.opt_results.EW)
                res = obj.opt_results.EW;
                if strcmp(dir, 'E')
                    idx = find(res.ids_E == vehicleID);
                    if ~isempty(idx)
                        tau = res.tau_E(idx);
                        return;
                    end
                elseif strcmp(dir, 'W')
                    idx = find(res.ids_W == vehicleID);
                    if ~isempty(idx)
                        tau = res.tau_W(idx);
                        return;
                    end
                end
            end
            
            % Check NS optimization results
            if isfield(obj.opt_results, 'NS') && ~isempty(obj.opt_results.NS)
                res = obj.opt_results.NS;
                if strcmp(dir, 'N')
                    idx = find(res.ids_N == vehicleID);
                    if ~isempty(idx)
                        tau = res.tau_N(idx);
                        return;
                    end
                elseif strcmp(dir, 'S')
                    idx = find(res.ids_S == vehicleID);
                    if ~isempty(idx)
                        tau = res.tau_S(idx);
                        return;
                    end
                end
            end
        end
        
        %% ==================== DEBUG / UTILITY ====================
        
        %% Print summary
        function PrintSummary(obj)
            fprintf('=== RSU %d Summary ===\n', obj.ID);
            fprintf('Position: (%.1f, %.1f)\n', obj.X, obj.Y);
            
            dirs = {'N', 'S', 'E', 'W'};
            for d = 1:length(dirs)
                dir = dirs{d};
                ids = obj.active_vehicles.(dir);
                fprintf('Direction %s: %d active vehicles\n', dir, length(ids));
            end
            
            if isfield(obj.opt_results, 'EW') && ~isempty(obj.opt_results.EW)
                fprintf('\nEW Optimization Result:\n');
                fprintf('  Optimal gap position: p = %d\n', obj.opt_results.EW.opt_p);
                fprintf('  Minimum cost: %.2f\n', obj.opt_results.EW.min_cost);
            end
            
            if isfield(obj.opt_results, 'NS') && ~isempty(obj.opt_results.NS)
                fprintf('\nNS Optimization Result:\n');
                fprintf('  Optimal gap position: p = %d\n', obj.opt_results.NS.opt_p);
                fprintf('  Minimum cost: %.2f\n', obj.opt_results.NS.min_cost);
            end
            
            fprintf('=====================\n');
        end
        
        %% Print arrival time estimates
        function PrintArrivalTimes(obj)
            fprintf('\n=== Arrival Time Estimates ===\n');
            dirs = {'N', 'S', 'E', 'W'};
            for d = 1:length(dirs)
                dir = dirs{d};
                [ids, taus] = obj.EstimateAllArrivalTimes(dir);
                if ~isempty(ids)
                    fprintf('Direction %s:\n', dir);
                    for i = 1:length(ids)
                        data = obj.GetVehicleData(ids(i));
                        turnStr = '';
                        if ~isempty(data) && data.TurnRight
                            turnStr = ' [RIGHT TURN]';
                        end
                        fprintf('  Vehicle %d: tau = %.2f s%s\n', ids(i), taus(i), turnStr);
                    end
                end
            end
            fprintf('==============================\n');
        end
    end
end