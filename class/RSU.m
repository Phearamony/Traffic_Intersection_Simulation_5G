classdef RSU<handle
    properties(GetAccess=public)
        ID=[];

        % V2X_data (received from vehicles)
        v2x_data = struct();
        str_v2x_data = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
        str_v2x_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());

        % X2V_data (sent to vehicles)
        x2v_data = struct();
        str_x2v_data = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
        str_x2v_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());

        active_vehicles = struct('N',[],'S',[],'E',[],'W',[]);
        opt_results = struct();
        coordinated_times = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
    end

    properties
        X=0; Y=0;
        SHORT_K = 20;
        COMM_RANGE = 200;
        INTERSECTION_RADIUS = 15;

        % Paper parameters
        tau_safe = 4.0;
        h_time = 2.0;
        omega_East = 1.0;
        omega_West = 1.0;
        omega_NS = 1.0;
        Vd = 25;
        dt_sim = 0.5;
        stop_line = 10;

        % IDM parameters
        a_max = 1.5;
        b_comfort = 2.5;
        s0 = 2.0;
        T_hw = 1.5;
        vehicle_length = 4.0;
    end

    methods
        function obj=RSU(ID, X, Y)
            obj.ID=ID; obj.Y=Y; obj.X=X;
        end
    end

    methods(Static)
        function H = initVehHistory()
            H = struct('t',[], 'X',[], 'Y',[], 'V',[], 'Ac',[], ...
                'TRight',[], 'TRighted',[], 'TLeft',[], 'TLefted',[]);
        end
        function H = initX2VHistory()
            H = struct('t',[], 'turn_signal',[], 'recommended_V',[], ...
                'recommended_Ac',[], 'priority',[], 'message',[], ...
                'target_arrival_time',[]);
        end
        function H = trimLastK(H, K)
            fn = fieldnames(H);
            for i = 1:numel(fn)
                v = H.(fn{i});
                if isnumeric(v), n=numel(v); if n>K, H.(fn{i})=v(n-K+1:n); end
                elseif iscell(v), n=numel(v); if n>K, H.(fn{i})=v(n-K+1:n); end
                end
            end
        end
    end

    methods
        %% V2X RECEIVING
        function ReceivingV2X(obj, data)
            vfield = ['v', num2str(data.ID)];
            dir = upper(char(data.Dir));
            obj.v2x_data.(vfield) = data;
            if ~ismember(data.ID, obj.active_vehicles.(dir))
                obj.active_vehicles.(dir)(end+1) = data.ID;
            end
            obj.StoreV2XData(data);
            obj.StoreV2XDataShort(data);
        end

        function StoreV2XData(obj, data)
            dir = upper(char(data.Dir));
            vfield = ['v', num2str(data.ID)];
            if ~isfield(obj.str_v2x_data.(dir), vfield)
                obj.str_v2x_data.(dir).(vfield) = RSU.initVehHistory();
            end
            H = obj.str_v2x_data.(dir).(vfield);
            H.t(end+1)=data.t; H.X(end+1)=data.X; H.Y(end+1)=data.Y;
            H.V(end+1)=data.V; H.Ac(end+1)=data.Ac;
            H.TRight(end+1)=double(data.TurnRight);
            H.TRighted(end+1)=double(data.TurnedRight);
            H.TLeft(end+1)=double(data.TurnLeft);
            H.TLefted(end+1)=double(data.TurnedLeft);
            obj.str_v2x_data.(dir).(vfield) = H;
        end

        function StoreV2XDataShort(obj, data)
            dir = upper(char(data.Dir));
            vfield = ['v', num2str(data.ID)];
            if ~isfield(obj.str_v2x_data_short.(dir), vfield)
                obj.str_v2x_data_short.(dir).(vfield) = RSU.initVehHistory();
            end
            Hs = obj.str_v2x_data_short.(dir).(vfield);
            Hs.t(end+1)=data.t; Hs.X(end+1)=data.X; Hs.Y(end+1)=data.Y;
            Hs.V(end+1)=data.V; Hs.Ac(end+1)=data.Ac;
            Hs.TRight(end+1)=double(data.TurnRight);
            Hs.TRighted(end+1)=double(data.TurnedRight);
            Hs.TLeft(end+1)=double(data.TurnLeft);
            Hs.TLefted(end+1)=double(data.TurnedLeft);
            Hs = RSU.trimLastK(Hs, obj.SHORT_K);
            obj.str_v2x_data_short.(dir).(vfield) = Hs;
        end

        function ResetV2XShort(obj)
            obj.str_v2x_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
        end

        %% X2V SENDING
        function data = SendingX2V(obj, t, vehicleID, turn_signal, recommended_V, recommended_Ac, priority, message, target_arrival_time)
            if nargin < 9, target_arrival_time = NaN; end
            vfield = ['v', num2str(vehicleID)];
            data = struct('t',t,'target_ID',vehicleID,'turn_signal',turn_signal,...
                'recommended_V',recommended_V,'recommended_Ac',recommended_Ac,...
                'priority',priority,'message',message,'target_arrival_time',target_arrival_time);
            obj.x2v_data.(vfield) = data;
            if isfield(obj.v2x_data, vfield) && isfield(obj.v2x_data.(vfield), 'Dir')
                dir = upper(char(obj.v2x_data.(vfield).Dir));
                obj.StoreX2VData(dir, vehicleID, data);
                obj.StoreX2VDataShort(dir, vehicleID, data);
            end
        end

        function StoreX2VData(obj, dir, vehicleID, data)
            vfield = ['v', num2str(vehicleID)];
            if ~isfield(obj.str_x2v_data.(dir), vfield)
                obj.str_x2v_data.(dir).(vfield) = RSU.initX2VHistory();
            end
            H = obj.str_x2v_data.(dir).(vfield);
            H.t(end+1)=data.t; H.turn_signal(end+1)=data.turn_signal;
            H.recommended_V(end+1)=data.recommended_V;
            H.recommended_Ac(end+1)=data.recommended_Ac;
            H.priority(end+1)=data.priority; H.message{end+1}=data.message;
            H.target_arrival_time(end+1)=data.target_arrival_time;
            obj.str_x2v_data.(dir).(vfield) = H;
        end

        function StoreX2VDataShort(obj, dir, vehicleID, data)
            vfield = ['v', num2str(vehicleID)];
            if ~isfield(obj.str_x2v_data_short.(dir), vfield)
                obj.str_x2v_data_short.(dir).(vfield) = RSU.initX2VHistory();
            end
            Hs = obj.str_x2v_data_short.(dir).(vfield);
            Hs.t(end+1)=data.t; Hs.turn_signal(end+1)=data.turn_signal;
            Hs.recommended_V(end+1)=data.recommended_V;
            Hs.recommended_Ac(end+1)=data.recommended_Ac;
            Hs.priority(end+1)=data.priority; Hs.message{end+1}=data.message;
            Hs.target_arrival_time(end+1)=data.target_arrival_time;
            Hs = RSU.trimLastK(Hs, obj.SHORT_K);
            obj.str_x2v_data_short.(dir).(vfield) = Hs;
        end

        function ResetX2VShort(obj)
            obj.str_x2v_data_short = struct('N',struct(),'S',struct(),'E',struct(),'W',struct());
        end
        function ResetAllShort(obj)
            obj.ResetV2XShort(); obj.ResetX2VShort();
        end

        %% VEHICLE MANAGEMENT
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
        function ids = GetActiveVehicles(obj, dir)
            ids = obj.active_vehicles.(upper(dir));
        end
        function data = GetVehicleData(obj, vehicleID)
            vfield = ['v', num2str(vehicleID)];
            if isfield(obj.v2x_data, vfield), data = obj.v2x_data.(vfield);
            else, data = []; end
        end

        %% ARRIVAL TIME ESTIMATION (MPC-BASED)
        function [sorted_ids, sorted_dists] = GetSortedVehiclesByDir(obj, dir)
            ids = obj.GetActiveVehicles(dir);
            n = length(ids);
            if n == 0, sorted_ids=[]; sorted_dists=[]; return; end
            dists = zeros(1, n);
            for i = 1:n
                d = obj.GetVehicleDataFromShort(ids(i), dir);
                if isempty(d), dists(i)=Inf; continue; end
                switch dir
                    case 'N', dists(i) = abs(-obj.stop_line - d.Y);
                    case 'S', dists(i) = abs(obj.stop_line - d.Y);
                    case 'E', dists(i) = abs(-obj.stop_line - d.X);
                    case 'W', dists(i) = abs(obj.stop_line - d.X);
                end
            end
            [sorted_dists, sortIdx] = sort(dists, 'ascend');
            sorted_ids = ids(sortIdx);
        end

        function before = IsBeforeStopLine(obj, data)
            switch data.Dir
                case 'N', before = data.Y < -obj.stop_line;
                case 'S', before = data.Y > obj.stop_line;
                case 'E', before = data.X < -obj.stop_line;
                case 'W', before = data.X > obj.stop_line;
                otherwise, before = true;
            end
        end

        function [tau, trajectory] = EstimateArrivalTimeMPC(obj, vehicleID, trafficLight, precedingData)
            tempData = obj.GetVehicleData(vehicleID);
            if isempty(tempData)
                tau = Inf; trajectory = []; return;
            end
            dir = tempData.Dir;

            % Use short storage for smoothed data
            data = obj.GetVehicleDataFromShort(vehicleID, dir);
            if isempty(data)
                tau = Inf; trajectory = []; return;
            end
            if isempty(data), tau=Inf; trajectory=[]; return; end
            dir = data.Dir;
            if strcmp(dir,'N')||strcmp(dir,'S'), lightState=trafficLight.NS;
            else, lightState=trafficLight.EW; end
            isRedOrYellow = strcmp(lightState,'red')||strcmp(lightState,'yellow');

            sim = struct('X',data.X,'Y',data.Y,'V',data.V,'Dir',dir);
            if nargin>=4 && ~isempty(precedingData)
                lead = struct('X',precedingData.X,'Y',precedingData.Y,'V',precedingData.V,'Dir',precedingData.Dir);
            else, lead = []; end

            dt=obj.dt_sim; max_time=60; t=0;
            trajectory = struct('t',[],'X',[],'Y',[],'V',[],'Ac',[]);

            while t < max_time
                trajectory.t(end+1)=t; trajectory.X(end+1)=sim.X;
                trajectory.Y(end+1)=sim.Y; trajectory.V(end+1)=sim.V;

                if obj.HasReachedStopLine(sim), tau=t; return; end

                isBeforeStopLine = obj.IsBeforeStopLineSim(sim);
                targetCar = obj.DetermineTarget(sim, lead, isRedOrYellow, isBeforeStopLine);
                acc = obj.CalculateIDMAcceleration(sim, targetCar);
                trajectory.Ac(end+1) = acc;

                sim.V = sim.V + acc*dt;
                if sim.V<0, sim.V=0; end
                delta = sim.V*dt + 0.5*acc*dt^2;
                if sim.V<0.001 && acc<0, delta=0; end

                switch sim.Dir
                    case 'N', sim.Y=sim.Y+delta;
                    case 'S', sim.Y=sim.Y-delta;
                    case 'E', sim.X=sim.X+delta;
                    case 'W', sim.X=sim.X-delta;
                end

                if ~isempty(lead)
                    ld = lead.V*dt;
                    switch lead.Dir
                        case 'N', lead.Y=lead.Y+ld;
                        case 'S', lead.Y=lead.Y-ld;
                        case 'E', lead.X=lead.X+ld;
                        case 'W', lead.X=lead.X-ld;
                    end
                end
                t = t + dt;
            end
            tau = Inf;
        end

        function reached = HasReachedStopLine(obj, sim)
            switch sim.Dir
                case 'N', reached = sim.Y >= -obj.stop_line;
                case 'S', reached = sim.Y <= obj.stop_line;
                case 'E', reached = sim.X >= -obj.stop_line;
                case 'W', reached = sim.X <= obj.stop_line;
                otherwise, reached = false;
            end
        end

        function before = IsBeforeStopLineSim(obj, sim)
            switch sim.Dir
                case 'N', before = sim.Y < -obj.stop_line;
                case 'S', before = sim.Y > obj.stop_line;
                case 'E', before = sim.X < -obj.stop_line;
                case 'W', before = sim.X > obj.stop_line;
                otherwise, before = true;
            end
        end

        function targetCar = DetermineTarget(obj, sim, lead, isRedOrYellow, isBeforeStopLine)
            if isRedOrYellow && isBeforeStopLine
                if ~isempty(lead) && obj.IsBeforeStopLineSim(lead)
                    targetCar = lead;
                else
                    targetCar = obj.CreateStopLineDummy(sim);
                end
            else
                if ~isempty(lead), targetCar = lead;
                else, targetCar = []; end
            end
        end

        function dummy = CreateStopLineDummy(obj, sim)
            dummy = struct('V',0,'Dir',sim.Dir);
            switch sim.Dir
                case 'N', dummy.X=sim.X; dummy.Y=-obj.stop_line+obj.vehicle_length;
                case 'S', dummy.X=sim.X; dummy.Y=obj.stop_line-obj.vehicle_length;
                case 'E', dummy.X=-obj.stop_line+obj.vehicle_length; dummy.Y=sim.Y;
                case 'W', dummy.X=obj.stop_line-obj.vehicle_length; dummy.Y=sim.Y;
            end
        end

        function acc = CalculateIDMAcceleration(obj, sim, targetCar)
            if isempty(targetCar)
                acc = obj.a_max*(1-(sim.V/obj.Vd)^4);
                acc = max(min(acc,2),-7); return;
            end
            switch sim.Dir
                case 'N', gap = targetCar.Y - sim.Y - obj.vehicle_length;
                case 'S', gap = sim.Y - targetCar.Y - obj.vehicle_length;
                case 'E', gap = targetCar.X - sim.X - obj.vehicle_length;
                case 'W', gap = sim.X - targetCar.X - obj.vehicle_length;
            end
            gap = max(gap, 0.5);
            dV = sim.V - targetCar.V;
            s_star = obj.s0 + sim.V*obj.T_hw + sim.V*dV/(2*sqrt(obj.a_max*obj.b_comfort));
            s_star = max(s_star, obj.s0);
            a_free = obj.a_max*(1-(sim.V/obj.Vd)^4);
            a_follow = -obj.a_max*(s_star/gap)^2;
            acc = a_free + a_follow;
            acc = max(min(acc,2),-7);
        end

        function [ids, taus] = EstimateAllArrivalTimesMPC(obj, dir, trafficLight)
            [sorted_ids, ~] = obj.GetSortedVehiclesByDir(dir);
            n = length(sorted_ids);
            if n==0, ids=[]; taus=[]; return; end
            taus = zeros(1, n);
            [taus(1), ~] = obj.EstimateArrivalTimeMPC(sorted_ids(1), trafficLight, []);
            for i = 2:n
                precedingData = obj.GetVehicleData(sorted_ids(i-1));
                [tau_follow, ~] = obj.EstimateArrivalTimeMPC(sorted_ids(i), trafficLight, precedingData);
                tau_headway = taus(i-1) + obj.h_time;
                taus(i) = max(tau_follow, tau_headway);
            end
            ids = sorted_ids;
        end

        %% RIGHT-TURN OPTIMIZATION
        function [opt_p, coordinated_tau_E, coordinated_tau_W, min_cost] = OptimizeRightTurn_EW(obj, turningCarID, trafficLight)
            turningData = obj.GetVehicleData(turningCarID);
            if isempty(turningData)||~turningData.TurnRight
                opt_p=0; coordinated_tau_E=[]; coordinated_tau_W=[]; min_cost=Inf; return;
            end
            [ids_E, tau_bar_E] = obj.EstimateAllArrivalTimesMPC('E', trafficLight);
            [ids_W, tau_bar_W] = obj.EstimateAllArrivalTimesMPC('W', trafficLight);
            M=length(ids_E); N=length(ids_W);

            fprintf('\n=== Right-Turn Opt (EW) ===\n');
            fprintf('E(%d): ',M); for i=1:M, d=obj.GetVehicleData(ids_E(i)); fprintf('%d(%.1fs) ',ids_E(i),tau_bar_E(i)); end; fprintf('\n');
            fprintf('W(%d): ',N); for i=1:N, d=obj.GetVehicleData(ids_W(i)); tr=''; if d.TurnRight,tr='*';end; fprintf('%d%s(%.1fs) ',ids_W(i),tr,tau_bar_W(i)); end; fprintf('\n');

            q = find(ids_W == turningCarID);
            if isempty(q), opt_p=0; coordinated_tau_E=tau_bar_E; coordinated_tau_W=tau_bar_W; min_cost=Inf; return; end

            straight_E_mask = true(1, M);
            for i=1:M, eData=obj.GetVehicleData(ids_E(i)); if ~isempty(eData)&&eData.TurnRight, straight_E_mask(i)=false; end; end

            min_cost=Inf; opt_p=M+1; best_tau_E=tau_bar_E; best_tau_W=tau_bar_W;
            for p = 1:(M+1)
                [tau_E, tau_W, feasible] = obj.ComputeCoordinatedTimes_EW(tau_bar_E, tau_bar_W, p, q, straight_E_mask);
                if ~feasible, continue; end
                delay_E = tau_E - tau_bar_E; delay_W = tau_W - tau_bar_W;
                cost = obj.omega_East*sum(delay_E.^2) + obj.omega_West*sum(delay_W.^2);
                fprintf('p=%d: cost=%.2f\n', p, cost);
                if cost < min_cost
                    min_cost=cost; opt_p=p; best_tau_E=tau_E; best_tau_W=tau_W;
                end
            end
            coordinated_tau_E=best_tau_E; coordinated_tau_W=best_tau_W;
            obj.opt_results.EW = struct('opt_p',opt_p,'tau_E',coordinated_tau_E,'tau_W',coordinated_tau_W,...
                'tau_bar_E',tau_bar_E,'tau_bar_W',tau_bar_W,'ids_E',ids_E,'ids_W',ids_W,...
                'turningCarID',turningCarID,'min_cost',min_cost);
            fprintf('=== OPTIMAL: p=%d, cost=%.2f ===\n', opt_p, min_cost);
        end

        function [tau_E, tau_W, feasible] = ComputeCoordinatedTimes_EW(obj, tau_bar_E, tau_bar_W, p, q, straight_E_mask)
            M=length(tau_bar_E); N=length(tau_bar_W);
            tau_E=tau_bar_E; tau_W=tau_bar_W; feasible=true;
            if p==1, tau_wq=tau_bar_W(q);
            else, tau_wq=max(tau_bar_W(q), tau_E(p-1)+obj.tau_safe); end
            tau_W(q)=tau_wq;
            if p<=M && straight_E_mask(p), tau_E(p)=max(tau_bar_E(p), tau_wq+obj.tau_safe); end
            for m=(p+1):M, if straight_E_mask(m), tau_E(m)=max(tau_bar_E(m), tau_E(m-1)+obj.h_time); end; end
            for n=(q+1):N, tau_W(n)=max(tau_bar_W(n), tau_W(n-1)+obj.h_time); end
            if any(tau_E<tau_bar_E-0.01)||any(tau_W<tau_bar_W-0.01), feasible=false; end
        end

        function [opt_p, coordinated_tau_S, coordinated_tau_N, min_cost] = OptimizeRightTurn_NS(obj, turningCarID, trafficLight)
            turningData = obj.GetVehicleData(turningCarID);
            if isempty(turningData)||~turningData.TurnRight, opt_p=0; coordinated_tau_S=[]; coordinated_tau_N=[]; min_cost=Inf; return; end
            [ids_S, tau_bar_S] = obj.EstimateAllArrivalTimesMPC('S', trafficLight);
            [ids_N, tau_bar_N] = obj.EstimateAllArrivalTimesMPC('N', trafficLight);
            M=length(ids_S); N=length(ids_N);
            q = find(ids_N == turningCarID);
            if isempty(q), opt_p=0; coordinated_tau_S=tau_bar_S; coordinated_tau_N=tau_bar_N; min_cost=Inf; return; end
            straight_S_mask = true(1, M);
            for i=1:M, sData=obj.GetVehicleData(ids_S(i)); if ~isempty(sData)&&sData.TurnRight, straight_S_mask(i)=false; end; end
            min_cost=Inf; opt_p=M+1; best_tau_S=tau_bar_S; best_tau_N=tau_bar_N;
            for p = 1:(M+1)
                [tau_S, tau_N, feasible] = obj.ComputeCoordinatedTimes_NS(tau_bar_S, tau_bar_N, p, q, straight_S_mask);
                if ~feasible, continue; end
                delay_S = tau_S - tau_bar_S; delay_N = tau_N - tau_bar_N;
                cost = obj.omega_NS*sum(delay_S.^2) + obj.omega_NS*sum(delay_N.^2);
                if cost < min_cost, min_cost=cost; opt_p=p; best_tau_S=tau_S; best_tau_N=tau_N; end
            end
            coordinated_tau_S=best_tau_S; coordinated_tau_N=best_tau_N;
            obj.opt_results.NS = struct('opt_p',opt_p,'tau_S',coordinated_tau_S,'tau_N',coordinated_tau_N,...
                'tau_bar_S',tau_bar_S,'tau_bar_N',tau_bar_N,'ids_S',ids_S,'ids_N',ids_N,...
                'turningCarID',turningCarID,'min_cost',min_cost);
        end

        function [tau_S, tau_N, feasible] = ComputeCoordinatedTimes_NS(obj, tau_bar_S, tau_bar_N, p, q, straight_S_mask)
            M=length(tau_bar_S); N=length(tau_bar_N);
            tau_S=tau_bar_S; tau_N=tau_bar_N; feasible=true;
            if p==1, tau_nq=tau_bar_N(q);
            else, tau_nq=max(tau_bar_N(q), tau_S(p-1)+obj.tau_safe); end
            tau_N(q)=tau_nq;
            if p<=M && straight_S_mask(p), tau_S(p)=max(tau_bar_S(p), tau_nq+obj.tau_safe); end
            for m=(p+1):M, if straight_S_mask(m), tau_S(m)=max(tau_bar_S(m), tau_S(m-1)+obj.h_time); end; end
            for n=(q+1):N, tau_N(n)=max(tau_bar_N(n), tau_N(n-1)+obj.h_time); end
            if any(tau_S<tau_bar_S-0.01)||any(tau_N<tau_bar_N-0.01), feasible=false; end
        end

        function [opt_p, coordinated_tau_W, coordinated_tau_E, min_cost] = OptimizeRightTurn_EW_Reverse(obj, turningCarID, trafficLight)
            turningData = obj.GetVehicleData(turningCarID);
            if isempty(turningData)||~turningData.TurnRight, opt_p=0; coordinated_tau_W=[]; coordinated_tau_E=[]; min_cost=Inf; return; end
            [ids_W, tau_bar_W] = obj.EstimateAllArrivalTimesMPC('W', trafficLight);
            [ids_E, tau_bar_E] = obj.EstimateAllArrivalTimesMPC('E', trafficLight);
            M=length(ids_W); N=length(ids_E);
            q = find(ids_E == turningCarID);
            if isempty(q), opt_p=0; coordinated_tau_W=tau_bar_W; coordinated_tau_E=tau_bar_E; min_cost=Inf; return; end
            straight_W_mask = true(1, M);
            for i=1:M, wData=obj.GetVehicleData(ids_W(i)); if ~isempty(wData)&&wData.TurnRight, straight_W_mask(i)=false; end; end
            min_cost=Inf; opt_p=M+1; best_tau_W=tau_bar_W; best_tau_E=tau_bar_E;
            for p = 1:(M+1)
                tau_W=tau_bar_W; tau_E=tau_bar_E;
                if p==1, tau_eq=tau_bar_E(q); else, tau_eq=max(tau_bar_E(q), tau_W(p-1)+obj.tau_safe); end
                tau_E(q)=tau_eq;
                if p<=M && straight_W_mask(p), tau_W(p)=max(tau_bar_W(p), tau_eq+obj.tau_safe); end
                for m=(p+1):M, if straight_W_mask(m), tau_W(m)=max(tau_bar_W(m), tau_W(m-1)+obj.h_time); end; end
                for n=(q+1):N, tau_E(n)=max(tau_bar_E(n), tau_E(n-1)+obj.h_time); end
                if any(tau_W<tau_bar_W-0.01)||any(tau_E<tau_bar_E-0.01), continue; end
                delay_W=tau_W-tau_bar_W; delay_E=tau_E-tau_bar_E;
                cost = obj.omega_West*sum(delay_W.^2) + obj.omega_East*sum(delay_E.^2);
                if cost<min_cost, min_cost=cost; opt_p=p; best_tau_W=tau_W; best_tau_E=tau_E; end
            end
            coordinated_tau_W=best_tau_W; coordinated_tau_E=best_tau_E;
        end

        function [opt_p, coordinated_tau_N, coordinated_tau_S, min_cost] = OptimizeRightTurn_NS_Reverse(obj, turningCarID, trafficLight)
            turningData = obj.GetVehicleData(turningCarID);
            if isempty(turningData)||~turningData.TurnRight, opt_p=0; coordinated_tau_N=[]; coordinated_tau_S=[]; min_cost=Inf; return; end
            [ids_N, tau_bar_N] = obj.EstimateAllArrivalTimesMPC('N', trafficLight);
            [ids_S, tau_bar_S] = obj.EstimateAllArrivalTimesMPC('S', trafficLight);
            M=length(ids_N); N=length(ids_S);
            q = find(ids_S == turningCarID);
            if isempty(q), opt_p=0; coordinated_tau_N=tau_bar_N; coordinated_tau_S=tau_bar_S; min_cost=Inf; return; end
            straight_N_mask = true(1, M);
            for i=1:M, nData=obj.GetVehicleData(ids_N(i)); if ~isempty(nData)&&nData.TurnRight, straight_N_mask(i)=false; end; end
            min_cost=Inf; opt_p=M+1; best_tau_N=tau_bar_N; best_tau_S=tau_bar_S;
            for p = 1:(M+1)
                tau_N=tau_bar_N; tau_S=tau_bar_S;
                if p==1, tau_sq=tau_bar_S(q); else, tau_sq=max(tau_bar_S(q), tau_N(p-1)+obj.tau_safe); end
                tau_S(q)=tau_sq;
                if p<=M && straight_N_mask(p), tau_N(p)=max(tau_bar_N(p), tau_sq+obj.tau_safe); end
                for m=(p+1):M, if straight_N_mask(m), tau_N(m)=max(tau_bar_N(m), tau_N(m-1)+obj.h_time); end; end
                for n=(q+1):N, tau_S(n)=max(tau_bar_S(n), tau_S(n-1)+obj.h_time); end
                if any(tau_N<tau_bar_N-0.01)||any(tau_S<tau_bar_S-0.01), continue; end
                delay_N=tau_N-tau_bar_N; delay_S=tau_S-tau_bar_S;
                cost = obj.omega_NS*sum(delay_N.^2) + obj.omega_NS*sum(delay_S.^2);
                if cost<min_cost, min_cost=cost; opt_p=p; best_tau_N=tau_N; best_tau_S=tau_S; end
            end
            coordinated_tau_N=best_tau_N; coordinated_tau_S=best_tau_S;
        end

        %% VELOCITY RECOMMENDATION
        function V_rec = CalculateRecommendedVelocity(obj, vehicleID, target_tau, current_t)
            tempData = obj.GetVehicleData(vehicleID);
            if isempty(tempData), V_rec = NaN; return; end
            data = obj.GetVehicleDataFromShort(vehicleID, tempData.Dir);
            if isempty(data), V_rec=NaN; return; end
            switch data.Dir
                case 'N', dist=abs(-obj.stop_line-data.Y);
                case 'S', dist=abs(obj.stop_line-data.Y);
                case 'E', dist=abs(-obj.stop_line-data.X);
                case 'W', dist=abs(obj.stop_line-data.X);
                otherwise, V_rec=NaN; return;
            end
            time_available = target_tau - current_t;
            if time_available<=0, V_rec=data.V; return; end
            V_rec = dist/time_available;
            V_rec = max(min(V_rec, obj.Vd*1.2), 0);
        end

        %% HIGH-LEVEL COORDINATION
        function RunGreenPhaseOptimization(obj, t, greenDir, trafficLight)
            % Clear previous optimization results for this direction
            if strcmp(greenDir, 'EW')
                obj.opt_results.EW = [];
            else
                obj.opt_results.NS = [];
            end

            if strcmp(greenDir, 'EW')
                ids_W = obj.GetActiveVehicles('W');
                for i=1:length(ids_W)
                    data=obj.GetVehicleData(ids_W(i));
                    if ~isempty(data) && data.TurnRight && ~data.TurnedRight
                        fprintf('\n[t=%.1f] Found W turner %d - running optimization\n', t, ids_W(i));
                        obj.OptimizeRightTurn_EW(ids_W(i), trafficLight);
                        break;
                    end
                end

                ids_E = obj.GetActiveVehicles('E');
                for i=1:length(ids_E)
                    data=obj.GetVehicleData(ids_E(i));
                    if ~isempty(data) && data.TurnRight && ~data.TurnedRight
                        fprintf('\n[t=%.1f] Found E turner %d - running optimization\n', t, ids_E(i));
                        obj.OptimizeRightTurn_EW_Reverse(ids_E(i), trafficLight);
                        break;
                    end
                end
            elseif strcmp(greenDir, 'NS')
                ids_N = obj.GetActiveVehicles('N');
                for i=1:length(ids_N)
                    data=obj.GetVehicleData(ids_N(i));
                    if ~isempty(data) && data.TurnRight && ~data.TurnedRight
                        fprintf('\n[t=%.1f] Found N turner %d - running optimization\n', t, ids_N(i));
                        obj.OptimizeRightTurn_NS(ids_N(i), trafficLight);
                        break;
                    end
                end
                ids_S = obj.GetActiveVehicles('S');
                for i=1:length(ids_S)
                    data=obj.GetVehicleData(ids_S(i));
                    if ~isempty(data) && data.TurnRight && ~data.TurnedRight
                        fprintf('\n[t=%.1f] Found S turner %d - running optimization\n', t, ids_S(i));
                        obj.OptimizeRightTurn_NS_Reverse(ids_S(i), trafficLight);
                        break;
                    end
                end
            end
        end

        function tau = GetCoordinatedArrivalTime(obj, vehicleID, dir)
            tau = NaN;
            if isfield(obj.opt_results, 'EW') && ~isempty(obj.opt_results.EW)
                res = obj.opt_results.EW;
                if strcmp(dir,'E') && isfield(res,'ids_E')
                    idx=find(res.ids_E==vehicleID); if ~isempty(idx), tau=res.tau_E(idx); return; end
                elseif strcmp(dir,'W') && isfield(res,'ids_W')
                    idx=find(res.ids_W==vehicleID); if ~isempty(idx), tau=res.tau_W(idx); return; end
                end
            end
            if isfield(obj.opt_results, 'NS') && ~isempty(obj.opt_results.NS)
                res = obj.opt_results.NS;
                if strcmp(dir,'N') && isfield(res,'ids_N')
                    idx=find(res.ids_N==vehicleID); if ~isempty(idx), tau=res.tau_N(idx); return; end
                elseif strcmp(dir,'S') && isfield(res,'ids_S')
                    idx=find(res.ids_S==vehicleID); if ~isempty(idx), tau=res.tau_S(idx); return; end
                end
            end
        end

        %% Decision
        function [vpos, turn_signal] = RightTurnOpt(obj, vehicleID, t)
            % Unified right-turn decision interface
            % Handles: mutual right-turn (no conflict) AND normal optimization
            % Returns: vpos (struct with recommendation), turn_signal (+1=go, -1=wait, 0=no opinion)

            global TrafficLight

            tempData = obj.GetVehicleData(vehicleID);
            if isempty(tempData) || ~tempData.TurnRight
                vpos = []; turn_signal = 0; return;
            end

            data = obj.GetVehicleDataFromShort(vehicleID, tempData.Dir);
            if isempty(data) || ~data.TurnRight
                vpos = [];
                turn_signal = 0;
                return;
            end

            dir = data.Dir;

            % ============================================================
            % STEP 1: Check for MUTUAL RIGHT-TURN (no conflict case)
            % ============================================================
            % Opposite right-turners don't conflict:
            %   W→N vs E→S : different quadrants
            %   N→E vs S→W : different quadrants

            oppositeDir = '';
            switch dir
                case 'W', oppositeDir = 'E';
                case 'E', oppositeDir = 'W';
                case 'N', oppositeDir = 'S';
                case 'S', oppositeDir = 'N';
            end

            if ~isempty(oppositeDir)
                opposingTurner = obj.FindFirstRightTurner(oppositeDir);
                if ~isempty(opposingTurner)
                    % Mutual right-turn detected - BOTH can proceed safely!
                    fprintf('[t=%.1f] MUTUAL RIGHT-TURN: %s(%d) and %s(%d) - BOTH PROCEED\n', ...
                        t, dir, vehicleID, oppositeDir, opposingTurner);

                    vpos = struct();
                    vpos.vehicle_id = vehicleID;
                    vpos.direction = dir;
                    vpos.action = 'mutual_turn';
                    vpos.recommended_V = 3.0;  % Safe turning speed
                    vpos.coordinated_tau = 0;
                    turn_signal = 1;  % PROCEED
                    return;
                end
            end

            % ============================================================
            % STEP 2: Normal optimization (single turner vs straight traffic)
            % ============================================================
            tau = NaN;

            if strcmp(dir, 'E') || strcmp(dir, 'W')
                % Check if we have valid optimization results
                hasValidOpt = isfield(obj.opt_results, 'EW') && ~isempty(obj.opt_results.EW);

                if hasValidOpt
                    if obj.opt_results.EW.turningCarID ~= vehicleID
                        hasValidOpt = false;
                    end
                end

                if ~hasValidOpt
                    if strcmp(dir, 'W')
                        obj.OptimizeRightTurn_EW(vehicleID, TrafficLight);
                    else
                        obj.OptimizeRightTurn_EW_Reverse(vehicleID, TrafficLight);
                    end
                end

                tau = obj.GetCoordinatedArrivalTime(vehicleID, dir);

            else  % N or S
                hasValidOpt = isfield(obj.opt_results, 'NS') && ~isempty(obj.opt_results.NS);

                if hasValidOpt
                    if obj.opt_results.NS.turningCarID ~= vehicleID
                        hasValidOpt = false;
                    end
                end

                if ~hasValidOpt
                    if strcmp(dir, 'N')
                        obj.OptimizeRightTurn_NS(vehicleID, TrafficLight);
                    else
                        obj.OptimizeRightTurn_NS_Reverse(vehicleID, TrafficLight);
                    end
                end

                tau = obj.GetCoordinatedArrivalTime(vehicleID, dir);
            end

            % ============================================================
            % STEP 3: Build response
            % ============================================================
            vpos = struct();
            vpos.vehicle_id = vehicleID;
            vpos.coordinated_tau = tau;
            vpos.direction = dir;

            if isnan(tau)
                turn_signal = 0;
                vpos.recommended_V = NaN;
                vpos.action = 'no_coordination';
            else
                V_rec = obj.CalculateRecommendedVelocity(vehicleID, tau, t);
                vpos.recommended_V = V_rec;

                time_to_turn = tau;

                if time_to_turn <= 0.5
                    turn_signal = 1;  % PROCEED
                    vpos.action = 'proceed';
                else
                    turn_signal = -1;  % WAIT
                    vpos.action = 'wait';
                end
            end
        end


        % Helper
        function data = GetVehicleDataFromShort(obj, vehicleID, dir)
            % Get vehicle data using SHORT storage history for realistic estimation
            % Returns smoothed/filtered data based on recent history

            vfield = ['v', num2str(vehicleID)];
            dir = upper(char(dir));

            % Check if we have short storage data
            if ~isfield(obj.str_v2x_data_short.(dir), vfield)
                % Fallback to latest data
                data = obj.GetVehicleData(vehicleID);
                return;
            end

            H = obj.str_v2x_data_short.(dir).(vfield);
            n = length(H.t);

            if n == 0
                data = obj.GetVehicleData(vehicleID);
                return;
            end

            % Use latest position (can't average position)
            latest_X = H.X(end);
            latest_Y = H.Y(end);
            latest_t = H.t(end);

            % === Smooth velocity using moving average ===
            if n >= 3
                % Use last 3-5 samples for smoothing
                k = min(5, n);
                smoothed_V = mean(H.V(end-k+1:end));
            else
                smoothed_V = H.V(end);
            end

            % === Estimate acceleration from velocity history ===
            if n >= 2
                % Use finite difference: a ≈ (v2 - v1) / (t2 - t1)
                dt_hist = H.t(end) - H.t(end-1);
                if dt_hist > 0
                    estimated_Ac = (H.V(end) - H.V(end-1)) / dt_hist;
                else
                    estimated_Ac = H.Ac(end);
                end

                % Smooth acceleration if we have more history
                if n >= 3
                    k = min(4, n-1);
                    acc_samples = zeros(1, k);
                    for i = 1:k
                        idx = n - i;
                        dt_i = H.t(idx+1) - H.t(idx);
                        if dt_i > 0
                            acc_samples(i) = (H.V(idx+1) - H.V(idx)) / dt_i;
                        else
                            acc_samples(i) = H.Ac(idx);
                        end
                    end
                    estimated_Ac = mean(acc_samples);
                end

                % Clamp to reasonable range
                estimated_Ac = max(min(estimated_Ac, 3.0), -7.0);
            else
                estimated_Ac = H.Ac(end);
            end

            % === Build output struct ===
            data = struct();
            data.ID = vehicleID;
            data.t = latest_t;
            data.X = latest_X;
            data.Y = latest_Y;
            data.V = smoothed_V;
            data.Ac = estimated_Ac;
            data.Dir = dir;

            % Get turn flags from latest
            data.TurnRight = H.TRight(end);
            data.TurnedRight = H.TRighted(end);
            data.TurnLeft = H.TLeft(end);
            data.TurnedLeft = H.TLefted(end);

            % === Optional: Estimate velocity trend ===
            if n >= 3
                % Linear regression on recent velocities to get trend
                k = min(5, n);
                t_recent = H.t(end-k+1:end) - H.t(end-k+1);  % Normalize time
                v_recent = H.V(end-k+1:end);

                % Simple linear fit: v = a*t + b
                % Slope gives acceleration trend
                if range(t_recent) > 0
                    p = polyfit(t_recent, v_recent, 1);
                    data.V_trend = p(1);  % Velocity trend (m/s²)
                else
                    data.V_trend = 0;
                end
            else
                data.V_trend = 0;
            end
        end

        function turnerID = FindFirstRightTurner(obj, dir)
            % Find the first right-turning vehicle in given direction
            % that hasn't turned yet and is near the wait point

            turnerID = [];
            ids = obj.GetActiveVehicles(dir);

            wait_threshold = 10;  % meters from center

            for i = 1:length(ids)
                data = obj.GetVehicleData(ids(i));

                if isempty(data)
                    continue;
                end

                % Check if this vehicle is a right-turner that hasn't turned yet
                if data.TurnRight && ~data.TurnedRight
                    % Check if near the wait/turn zone
                    switch dir
                        case 'E', atWait = data.X >= -wait_threshold;
                        case 'W', atWait = data.X <= wait_threshold;
                        case 'N', atWait = data.Y >= -wait_threshold;
                        case 'S', atWait = data.Y <= wait_threshold;
                        otherwise, atWait = false;
                    end

                    if atWait
                        turnerID = ids(i);
                        return;
                    end
                end
            end
        end

        %% DEBUG
        function PrintSummary(obj)
            fprintf('=== RSU %d ===\n', obj.ID);
            dirs = {'N','S','E','W'};
            for d=1:4, fprintf('%s:%d ', dirs{d}, length(obj.active_vehicles.(dirs{d}))); end
            fprintf('\n');
            if isfield(obj.opt_results,'EW')&&~isempty(obj.opt_results.EW)
                fprintf('EW: p=%d cost=%.2f\n', obj.opt_results.EW.opt_p, obj.opt_results.EW.min_cost);
            end
            if isfield(obj.opt_results,'NS')&&~isempty(obj.opt_results.NS)
                fprintf('NS: p=%d cost=%.2f\n', obj.opt_results.NS.opt_p, obj.opt_results.NS.min_cost);
            end
        end

        function PrintArrivalTimes(obj, trafficLight)
            fprintf('\n=== Arrival Times ===\n');
            dirs = {'N','S','E','W'};
            for d=1:4
                dir=dirs{d};
                [ids, taus] = obj.EstimateAllArrivalTimesMPC(dir, trafficLight);
                if ~isempty(ids)
                    fprintf('%s: ', dir);
                    for i=1:length(ids)
                        data=obj.GetVehicleData(ids(i)); tr='';
                        if ~isempty(data)&&data.TurnRight, tr='*'; end
                        fprintf('%d%s(%.1fs) ', ids(i), tr, taus(i));
                    end
                    fprintf('\n');
                end
            end
        end
    end
end