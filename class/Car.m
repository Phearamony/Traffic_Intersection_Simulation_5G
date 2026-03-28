classdef Car<handle
    properties(GetAccess=public)
        ID=[];
    end
    properties
        LCng=0;
        LcV=20;
        Clr=1;
        Lpf=10;
        X=0; Y=0;V=20; A=[0;0;0]; Ac=0;
        Vd=25; Th=1.3; R0=4.0;
        Lpos=0;

        % Turning
        Dir='N';
        TurnLeft = 0;
        TurnedLeft = 0;
        TurnRight = 0;
        TurnedRight = 0;
        Turn_signal = 0;

        % V2X_data (data this car sends to RSU)
        v2x_data = struct();
        str_v2x_data = struct();         % permanent storage of sent data
        str_v2x_data_short = struct();   % temp storage of sent data

        % X2V_data (data this car receives from RSU)
        x2v_data = struct();
        str_x2v_data = struct();         % permanent storage of received data
        str_x2v_data_short = struct();   % temp storage of received data

        % Short storage config
        SHORT_K = 20;  % keep last K entries in short storage
    end

    methods
        function obj=Car(ID, X, Y)
            global Vd;
            obj.ID=ID;
            obj.Y=Y;
            obj.Th=1.2+1.2*rand;
            obj.Vd=21+6*rand+2;
            obj.X=X;

            if rand < 0.25 && obj.TurnRight == 0  % 25% chance to turn right
                obj.TurnLeft = 1;
            end

            if rand < 0.15 && obj.TurnLeft == 0  % 25% chance to turn left
                obj.TurnRight = 1;
            end

            % Initialize storage structures
            obj.str_v2x_data = Car.initCarHistory();
            obj.str_v2x_data_short = Car.initCarHistory();
            obj.str_x2v_data = Car.initX2VHistory();
            obj.str_x2v_data_short = Car.initX2VHistory();

        end

    end

    methods(Static)
        %% IDM
        function Acel = IDM(H, P)
            % Calculate correct gap based on direction
            switch H.Dir
                case 'E'  % WEST → EAST (X increases)
                    S = P.X - H.X - H.R0;
                case 'W'  % EAST → WEST (X decreases)
                    S = H.X - P.X - H.R0;

                case 'N'  % SOUTH → NORTH (Y increases)
                    S = P.Y - H.Y - H.R0;
                case 'S'  % NORTH → SOUTH (Y decreases)
                    S = H.Y - P.Y - H.R0;
                otherwise
                    warning("Unknown direction: %s", H.Dir);
                    S = 10;
            end

            % Clamp small gap
            S = max(S, 0.95);

            dV = H.V - P.V;
            S0 = 1.0;
            a = 1.5; b = 2.5;
            Ss = S0 + H.V * H.Th + H.V * dV / (2 * sqrt(a * b));

            f = a * (1 - (H.V / H.Vd)^4 - (Ss / S)^2);
            f = 8 * tanh(f / 8.0) * (1.0 + randn / 30.0);
            Acel = f;
        end

        %% MPC
        % function Acel = MPC(H, P)
        %     global dt;
        %     %LL=L0;
        %     Umn=-7;Umx=2; %min and max acc
        %     T=4; %horizon : span it needs to predict like this one is X step ahead must be bigger than 4
        %     cx=3*(T+1)-1; %% X,V and A total dimension
        %     %ub=[]; lb=[];
        %     lb=nan(cx,1);
        %     ub=lb;
        %     lb(1:T+1)=-Inf;  %% Position limits
        %     ub(1:T+1)=Inf;
        %     lb(T+2:2*T+2)=0; %% minimum speed
        %     ub(T+2:2*T+2)=18; %% maximu speed
        %     lb(2*T+3:end)=Umn; %% minimum acceleration
        %     ub(2*T+3:end)=Umx; %% maximum acceleration
        %
        %     % equality
        %     Ax=1.0; Bx=dt; dt2=0.5*dt^2;
        %     Aeq= zeros(cx,cx);
        %
        %     T1=-eye(T+1); T1(2:end,1:T)=eye(T)*Ax+T1(2:end,1:T); % x_{k+1}-x_k terms
        %     Aeq(1:T+1,1:T+1)=T1; % x_{k+1}-x_k terms
        %     Aeq(T+2:2*T+2,T+2:2*T+2)=T1; % x_{k+1}-x_k terms
        %     Aeq(2:T+1,T+2:2*T+1)=eye(T)*Bx;  % + dt * v_k
        %     Aeq(2:T+1,2*T+3:3*T+2)=eye(T)*dt2; % + 0.5*dt^2 * u_k
        %     Aeq(T+3:2*T+2,2*T+3:3*T+2)=eye(T)*dt;
        %     xi=zeros(cx,1); x=xi;
        %
        %     Beq=zeros(cx,1);
        %
        %     switch H.Dir
        %         case 'E'  % WEST → EAST (X increases)
        %             xi(1)=H.X;  xi(T+2)=H.V;
        %             Beq(1)=-H.X; Beq(T+2)=-H.V;
        %             XPT=[[P.X+((1:T)*P.V*dt)]']; % predicted positions of the preceding (lead) vehicle
        %         case 'W'  % EAST → WEST (X decreases)
        %             xi(1)=H.X;  xi(T+2)=H.V;
        %             Beq(1)=-H.X; Beq(T+2)=-H.V;
        %             XPT=[[P.X+((1:T)*P.V*dt)]']; % predicted positions of the preceding (lead) vehicle
        %
        %         case 'N'  % SOUTH → NORTH (Y increases)
        %             xi(1)=H.Y;  xi(T+2)=H.V;
        %             Beq(1)=-H.Y; Beq(T+2)=-H.V;
        %             XPT=[[P.Y+((1:T)*P.V*dt)]']; % predicted positions of the preceding (lead) vehicle
        %         case 'S'  % NORTH → SOUTH (Y decreases)
        %             xi(1)=H.Y;  xi(T+2)=H.V;
        %             Beq(1)=-H.Y; Beq(T+2)=-H.V;
        %             XPT=[[P.Y+((1:T)*P.V*dt)]']; % predicted positions of the preceding (lead) vehicle
        %     end
        %
        %     % xi(1)=H.X;  xi(T+2)=H.V;
        %     % Beq(1)=-H.X; Beq(T+2)=-H.V;
        %     % XPT=[[P.X+((1:T)*P.V*dt)]']; % predicted positions of the preceding (lead) vehicle
        %
        %     % Initial Guess
        %     a0=-0.5;
        %     for J=1:T
        %         xi(J+1)= xi(J)+ xi(T+1+J)*dt+0.5*a0*dt^2;
        %         xi(T+1+J+1)=xi(T+1+J)+a0*dt;
        %     end
        %     xi(2*(T+1)+1:end)=a0;
        %     b=zeros(T,1);
        %     S0 = 1.0;
        %     b(1:T,1)=XPT-S0; % Gap with PV
        %     b(1:3,1)=b(1:3,1)+4;  % Extra safety for first 3 steps
        %
        %     A=zeros(T,cx);
        %     A(1:T,2:T+1)=eye(T); % v_{k+1}-v_k
        %     A(1:T,T+3:2*T+2)=0.5*eye(T); % + dt * u_k
        %
        %     % Optimize
        %
        %     options=optimset('Algorithm','sqp','GradObj','off');
        %     [x,~,~,~]=fmincon(@ObjFn,xi,A,b,Aeq,Beq,lb,ub);%,@NonLinCons,options
        %
        %     Acel = x(2*T+3);
        %
        %     %% MPC helper
        %     function [ Fn ] = ObjFn(x)
        %         global Vd
        %         %gradF=0;
        %
        %         Verr=Vd-x(T+2:2*(T+1)); %velocity error: (desired velocity) - velocity
        %         U=x(2*T+3:end); %control input: control aceleration
        %
        %         Fn=0.15*sum(Verr.^2)+9*sum(U.^2); %weight determines the importance of minimizing
        %     end
        %
        %     function [nc, ceq] = NonLinCons(x)
        %         ceq=[];
        %         nc=[];
        %         DCeq=[];
        %         DC=[];
        %     end
        %
        % end

        function Acel = MPC(H, P)
            % DROP-IN OPTIMIZED MPC (QP, condensed form, sparse, warm-start)
            % Interface unchanged: Acel = MPC(H,P)
            % Uses globals: dt, Vd

            % --- Globals / params (kept same semantics as your code) ---
            global dt
            Umn = -7; Umx = 2;          % accel bounds
            Vmn = 0;  Vmx = max(H.Vd, H.V) + 5;         % speed bounds
            T   = 8;                    % horizon (should be > 4 for it to stop in time)
            wv  = 0.15; wu = 9.0;       % cost weights (keep same)
            S0  = 1.0;                  % desired spacing to lead
            safetyBoost = 4;            % extra margin first 3 steps

            DEBUG = true;  % Set to false to disable debug output

            % --- 1D reduction (position along travel axis) ---
            switch H.Dir
                case 'E'
                    s0 = H.X;  v0 = H.V;
                    sL0 = P.X; vL = P.V;
                    gap_real = P.X - H.X;  % for debug
                case 'W'
                    s0 = -H.X;  v0 = H.V;
                    sL0 = -P.X; vL = P.V;
                    gap_real = H.X - P.X;  % for debug
                case 'N'
                    s0 = H.Y;  v0 = H.V;
                    sL0 = P.Y; vL = P.V;
                    gap_real = P.Y - H.Y;  % for debug
                case 'S'
                    s0 = -H.Y;  v0 = H.V;
                    sL0 = -P.Y; vL = P.V;
                    gap_real = H.Y - P.Y;  % for debug
                otherwise
                    warning('Unknown direction: %s', H.Dir);
                    Acel = 0; return;
            end

            if DEBUG && P.ID == -1  % Only debug when following dummy (stop line)
                fprintf('\n=== MPC DEBUG (Car %d, Dir=%s) ===\n', H.ID, H.Dir);
                fprintf('Host: s0=%.2f, v0=%.2f, Vd=%.2f\n', s0, v0, H.Vd);
                fprintf('Lead: sL0=%.2f, vL=%.2f (ID=%d)\n', sL0, vL, P.ID);
                fprintf('Real gap: %.2f m\n', gap_real);
                fprintf('Transformed gap: %.2f m\n', sL0 - s0);
            end

            % lead position prediction (constant vL, same as your XPT)
            sLead = sL0 + (1:T)' * (vL*dt);
            gapBoost = zeros(T,1); gapBoost(1:min(3,T)) = safetyBoost;

            if DEBUG && P.ID == -1
                fprintf('Lead predictions (first 3): [%.2f, %.2f, %.2f]\n', ...
                    sLead(1), sLead(2), sLead(3));
            end

            % --- Discrete model ---
            A = [1 dt; 0 1];
            B = [0.5*dt^2; dt];

            % --- Build (and cache) time-invariant prediction matrices ---
            % v = Av + S_v * U,   s = As + S_s * U
            persistent T_c dt_c S_v S_s H_qp Q_u Ak_pows U_warm
            if isempty(T_c) || T_c~=T || isempty(dt_c) || dt_c~=dt
                % Precompute A^k
                Ak_pows = cell(T+1,1);
                Ak_pows{1} = eye(2);            % A^0
                Ak = eye(2);
                for k=1:T
                    Ak = A*Ak;       % A^k
                    Ak_pows{k+1} = Ak;
                end
                e1 = [1 0]; e2 = [0 1];

                S_v = zeros(T,T);
                S_s = zeros(T,T);
                for k=1:T
                    for i=1:k
                        % Aki = (i==k) * eye(2) + (i<k) * Ak_pows{(k-i) + 1};
                        Aki = Ak_pows{(k - i) + 1};
                        S_v(k,i) = e2 * (Aki * B);
                        S_s(k,i) = e1 * (Aki * B);
                    end
                end

                % Cost Hessian (state-eliminated): 0.5*U'HU + f'U
                Qv = wv*speye(T);  R = wu*speye(T);
                H_qp = 2*(S_v.'*Qv*S_v + R);
                Q_u  = 2*S_v.'*Qv;              % multiplies (Av - Vd_vec)

                % Cache + make sparse
                S_v  = sparse(S_v);  S_s = sparse(S_s);
                H_qp = sparse(H_qp); Q_u = sparse(Q_u);

                T_c = T; dt_c = dt;
                U_warm = zeros(T,1);
            end

            % --- State-dependent parts (cheap each call) ---
            % Av = v due to initial (no control), As = s due to initial (no control)
            Av = zeros(T,1);  As = zeros(T,1);
            for k=1:T
                Av(k) = [0 1]*(Ak_pows{k+1}*[s0; v0]);
                As(k) = [1 0]*(Ak_pows{k+1}*[s0; v0]);
            end

            if DEBUG && P.ID == -1
                fprintf('Free evolution s (first 3): [%.2f, %.2f, %.2f]\n', ...
                    As(1), As(2), As(3));
                fprintf('Gap constraint target (first 3): [%.2f, %.2f, %.2f]\n', ...
                    sLead(1)-S0-gapBoost(1), sLead(2)-S0-gapBoost(2), sLead(3)-S0-gapBoost(3));
            end

            Vd_vec = H.Vd*ones(T,1);

            % --- QP: min 0.5 U'HU + f'U ---
            f_qp = Q_u * (Av - Vd_vec);

            % Speed bounds: Vmn <= Av + S_v U <= Vmx
            Aineq = [ S_v;  -S_v ];
            bineq = [ Vmx*ones(T,1) - Av;
                -(Vmn*ones(T,1) - Av) ];

            % Gap constraint: s_k <= sLead_k - (S0 + gapBoost_k)
            % i.e., As + S_s U <= sLead - S0 - gapBoost
            Aineq = [Aineq;  S_s];
            bineq = [bineq;  (sLead - (S0 + gapBoost)) - As];

            if DEBUG && P.ID == -1
                gap_slack = bineq(end-T+1:end);  % Last T constraints are gap constraints
                fprintf('Gap slack (first 3): [%.2f, %.2f, %.2f]\n', ...
                    gap_slack(1), gap_slack(2), gap_slack(3));
                fprintf('  (negative = constraint violated, must brake!)\n');
            end

            % Accel bounds
            lb = Umn*ones(T,1);
            ub = Umx*ones(T,1);

            % --- Solve QP (warm-started) ---
            opts = optimoptions('quadprog', ...
                'Algorithm','interior-point-convex', ...
                'Display','off', ...
                'MaxIterations',100, ...
                'OptimalityTolerance',1e-6, ...
                'ConstraintTolerance',1e-6);

            % If quadprog missing, graceful degrade
            solverAvailable = exist('quadprog','file')==2;
            if solverAvailable
                [U,~,exitflag] = quadprog(H_qp, f_qp, Aineq, bineq, [], [], lb, ub, U_warm, opts);
                if DEBUG && P.ID == -1
                    fprintf('QP exitflag: %d (1=success, <=0=failed)\n', exitflag);
                end
            else
                warning('quadprog not available, using fallback controller');
                U = U_warm; exitflag = 1; % fallback: keep last
            end

            if exitflag <= 0 || any(~isfinite(U))
                % Very occasional infeasibility: soft fallback to small accel toward Vd
                % QP FAILED - Determine safe fallback
                current_gap = sL0 - s0;
                stopping_dist = v0^2 / (2 * abs(Umn) + 0.01);

                % If we're anywhere near needing to stop, BRAKE
                if current_gap < stopping_dist * 1.5 + S0 + 20
                    U(1) = Umn;  % Maximum braking
                else
                    % Safe distance - gentle speed control
                    U(1) = max(min(0.3*(H.Vd - v0), Umx), Umn);
                end

                if DEBUG && P.ID == -1
                    fprintf('QP FAILED! gap=%.1f, stop_dist=%.1f, fallback U(1)=%.2f\n', ...
                        current_gap, stopping_dist, U(1));
                end
            end

            % Update warm-start (shift)
            U_warm = [U(2:end); U(end)];

            % Apply first control
            Acel = U(1);

            % Clamp to bounds (safety check)
            Acel = min(max(Acel, Umn), Umx);

            if DEBUG && P.ID == -1
                fprintf('OUTPUT Acceleration: %.2f m/s^2\n', Acel);
                fprintf('Expected stopping distance: %.2f m\n', v0^2/(2*abs(Acel)+0.01));
                fprintf('==============================\n\n');
            end
        end


        %% update position
        function fdp(obj)
            global dt;
            delta = obj.V * dt + 0.5 * obj.Ac * dt^2;
            if obj.V < 0.001 && obj.Ac < 0
                delta = 0;
            end

            switch obj.Dir
                case 'N'
                    obj.Y = obj.Y + delta;
                case 'S'
                    obj.Y = obj.Y - delta;
                case 'E'
                    obj.X = obj.X + delta;
                case 'W'
                    obj.X = obj.X - delta;
            end
        end

        % function Xnew=fdx(obj)
        %     global dt; global tr;
        %     Xnew=obj.X+obj.V*dt+0.5*obj.Ac*dt^2;
        %     if obj.V<0.001 &&obj.Ac<0
        %         Xnew=obj.X;
        %     end
        % end

        %% update velocity
        function Vnew=fdv(obj)
            global dt;
            % global tr;
            Vnew = obj.V + obj.Ac * dt;
            if Vnew < 0.001
                Vnew = 0;
            end
        end

        %% Initialize history structure for V2X (what car sends)
        function H = initCarHistory()
            H = struct('t',[], 'X',[], 'Y',[], 'V',[], 'Ac',[], ...
                       'TurnRight',[], 'TurnedRight',[], ...
                       'TurnLeft',[], 'TurnedLeft',[]);
        end

        %% Initialize history structure for X2V (what car receives)
        function H = initX2VHistory()
            H = struct('t',[], 'turn_signal',[], 'recommended_V',[], ...
                       'recommended_Ac',[], 'priority',[], 'message',[]);
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
        
        %% ==================== V2X COMMUNICATION ====================
        
        %% Send V2X data to RSU
        % Returns data packet that should be passed to RSU.ReceivingV2X()
        function data = SendingV2X(obj, t)
            % Create data packet
            obj.v2x_data = struct(...
                't', t, ...
                'ID', obj.ID, ...
                'X', obj.X, ...
                'Y', obj.Y, ...
                'V', obj.V, ...
                'Ac', obj.Ac, ...
                'Dir', obj.Dir, ...
                'TurnRight', obj.TurnRight, ...
                'TurnedRight', obj.TurnedRight, ...
                'TurnLeft', obj.TurnLeft, ...
                'TurnedLeft', obj.TurnedLeft ...
            );
            
            % Store in permanent history
            obj.StoreV2XSent(obj.v2x_data);
            
            % Store in temp (short) history
            obj.StoreV2XSentShort(obj.v2x_data);
            
            data = obj.v2x_data;
        end

        %% Store sent V2X data (permanent)
        function StoreV2XSent(obj, data)
            obj.str_v2x_data.t(end+1) = data.t;
            obj.str_v2x_data.X(end+1) = data.X;
            obj.str_v2x_data.Y(end+1) = data.Y;
            obj.str_v2x_data.V(end+1) = data.V;
            obj.str_v2x_data.Ac(end+1) = data.Ac;
            obj.str_v2x_data.TurnRight(end+1) = double(data.TurnRight);
            obj.str_v2x_data.TurnedRight(end+1) = double(data.TurnedRight);
            obj.str_v2x_data.TurnLeft(end+1) = double(data.TurnLeft);
            obj.str_v2x_data.TurnedLeft(end+1) = double(data.TurnedLeft);
        end

        %% Store sent V2X data (temp/short - keeps last K)
        function StoreV2XSentShort(obj, data)
            obj.str_v2x_data_short.t(end+1) = data.t;
            obj.str_v2x_data_short.X(end+1) = data.X;
            obj.str_v2x_data_short.Y(end+1) = data.Y;
            obj.str_v2x_data_short.V(end+1) = data.V;
            obj.str_v2x_data_short.Ac(end+1) = data.Ac;
            obj.str_v2x_data_short.TurnRight(end+1) = double(data.TurnRight);
            obj.str_v2x_data_short.TurnedRight(end+1) = double(data.TurnedRight);
            obj.str_v2x_data_short.TurnLeft(end+1) = double(data.TurnLeft);
            obj.str_v2x_data_short.TurnedLeft(end+1) = double(data.TurnedLeft);
            
            % Trim to last K entries
            obj.str_v2x_data_short = Car.trimLastK(obj.str_v2x_data_short, obj.SHORT_K);
        end

        %% Reset short V2X storage (call from main when needed)
        function ResetV2XShort(obj)
            obj.str_v2x_data_short = Car.initCarHistory();
        end

        %% ============== X2V Communication ==============

        %% Receive X2V data from RSU
        % data should contain: t, turn_signal, recommended_V, recommended_Ac, priority, message
        function ReceivingX2V(obj, data)
            obj.x2v_data = data;
            
            % Apply turn signal if provided
            if isfield(data, 'turn_signal') && ~isempty(data.turn_signal)
                obj.Turn_signal = data.turn_signal;
            end
            
            % Store in permanent history
            obj.StoreX2VReceived(data);
            
            % Store in temp (short) history
            obj.StoreX2VReceivedShort(data);
        end

        %% Store received X2V data (permanent)
        function StoreX2VReceived(obj, data)
            obj.str_x2v_data.t(end+1) = data.t;
            
            if isfield(data, 'turn_signal')
                obj.str_x2v_data.turn_signal(end+1) = data.turn_signal;
            else
                obj.str_x2v_data.turn_signal(end+1) = NaN;
            end
            
            if isfield(data, 'recommended_V')
                obj.str_x2v_data.recommended_V(end+1) = data.recommended_V;
            else
                obj.str_x2v_data.recommended_V(end+1) = NaN;
            end
            
            if isfield(data, 'recommended_Ac')
                obj.str_x2v_data.recommended_Ac(end+1) = data.recommended_Ac;
            else
                obj.str_x2v_data.recommended_Ac(end+1) = NaN;
            end
            
            if isfield(data, 'priority')
                obj.str_x2v_data.priority(end+1) = data.priority;
            else
                obj.str_x2v_data.priority(end+1) = NaN;
            end
            
            if isfield(data, 'message')
                obj.str_x2v_data.message{end+1} = data.message;
            else
                obj.str_x2v_data.message{end+1} = '';
            end
        end

        %% Store received X2V data (temp/short - keeps last K)
        function StoreX2VReceivedShort(obj, data)
            obj.str_x2v_data_short.t(end+1) = data.t;
            
            if isfield(data, 'turn_signal')
                obj.str_x2v_data_short.turn_signal(end+1) = data.turn_signal;
            else
                obj.str_x2v_data_short.turn_signal(end+1) = NaN;
            end
            
            if isfield(data, 'recommended_V')
                obj.str_x2v_data_short.recommended_V(end+1) = data.recommended_V;
            else
                obj.str_x2v_data_short.recommended_V(end+1) = NaN;
            end
            
            if isfield(data, 'recommended_Ac')
                obj.str_x2v_data_short.recommended_Ac(end+1) = data.recommended_Ac;
            else
                obj.str_x2v_data_short.recommended_Ac(end+1) = NaN;
            end
            
            if isfield(data, 'priority')
                obj.str_x2v_data_short.priority(end+1) = data.priority;
            else
                obj.str_x2v_data_short.priority(end+1) = NaN;
            end
            
            if isfield(data, 'message')
                obj.str_x2v_data_short.message{end+1} = data.message;
            else
                obj.str_x2v_data_short.message{end+1} = '';
            end
            
            % Trim to last K entries
            obj.str_x2v_data_short = Car.trimLastK(obj.str_x2v_data_short, obj.SHORT_K);
        end

        %% Reset short X2V storage (call from main when needed)
        function ResetX2VShort(obj)
            obj.str_x2v_data_short = Car.initX2VHistory();
        end

        %% Reset all short storage
        function ResetAllShort(obj)
            obj.ResetV2XShort();
            obj.ResetX2VShort();
        end

        %% Get last received X2V data
        function data = GetLastX2V(obj)
            data = obj.x2v_data;
        end

        %% Check if car has pending turn signal from RSU
        function has = HasTurnSignal(obj)
            has = ~isempty(obj.Turn_signal) && obj.Turn_signal ~= 0;
        end
    end
end

