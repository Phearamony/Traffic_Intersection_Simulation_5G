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
        Recieved_data = struct();

        % V2X_data
        v2x_data = struct();
        str_v2x_data = struct();
        str_v2x_data_short = struct();

        % X2V_data
        x2v_data = struct();
        str_x2v_data = struct();
        str_x2v_data_short = struct();

    end
    methods
        function obj=Car(ID, X, Y)
            global Vd;
            obj.ID=ID;
            obj.Y=Y;
            obj.Th=1.2+1.2*rand;
            obj.Vd=21+6*rand+2;
            obj.X=X;

            if rand < 0.25 && obj.TurnRight == 0  % 25% chance to turn left
                obj.TurnLeft = 1;
            end

            if rand < 0.15 && obj.TurnLeft == 0  % 25% chance to turn left
                obj.TurnRight = 1;
            end

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
            global dt Vd
            Umn = -7; Umx = 2;          % accel bounds
            Vmn = 0;  Vmx = 18;         % speed bounds
            T   = 15;                    % horizon (should be > 4 for it to stop in time)
            wv  = 0.15; wu = 9.0;       % cost weights (keep same)
            S0  = 1.0;                  % desired spacing to lead
            safetyBoost = 4;            % extra margin first 3 steps

            % --- 1D reduction (position along travel axis) ---
            switch H.Dir
                case 'E'
                    s0 = H.X;  v0 = H.V;
                    sL0 = P.X; vL = P.V;
                case 'W'
                    s0 = -H.X;  v0 = -H.V;
                    sL0 = -P.X; vL = -P.V;
                case 'N'
                    s0 = H.Y;  v0 = H.V;
                    sL0 = P.Y; vL = P.V;
                case 'S'
                    s0 = -H.Y;  v0 = -H.V;
                    sL0 = -P.Y; vL = -P.V;
                otherwise
                    Acel = 0; return;
            end
            % lead position prediction (constant vL, same as your XPT)
            sLead = sL0 + (1:T)' * (vL*dt);
            gapBoost = zeros(T,1); gapBoost(1:min(3,T)) = safetyBoost;

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
            Vd_vec = Vd*ones(T,1);

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
            else
                U = U_warm; exitflag = 1; % fallback: keep last
            end

            if exitflag <= 0 || any(~isfinite(U))
                % Very occasional infeasibility: soft fallback to small accel toward Vd
                U = U_warm;
                U(1) = 0.5*(Vd - v0);                 % gentle stabilizer
                U(1) = min(max(U(1), Umn), Umx);      % respect bounds
            end

            % Update warm-start (shift)
            U_warm = [U(2:end); U(end)];

            % Apply first control
            Acel = U(1);
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
    end
    
    methods
        %% Sending V2X_Data
        function data = SendingV2X(obj, t)
            obj.v2x_data = struct('t', t, 'ID', obj.ID, 'X', obj.X, 'Y', obj.Y, 'V', obj.V, 'Ac', obj.Ac, ...
                'Dir', obj.Dir, 'TurnRight', obj.TurnRight, 'TurnedRight', obj.TurnedRight, 'TurnLeft', obj.TurnLeft, 'TurnedLeft', obj.TurnedLeft);
            data = obj.v2x_data;
        end

        % since data = vpos + turn_signal --> use if turn_signal is empty
        % don't import it
        %% Recieving X2V_Data
        function RecievingX2V(obj, data)
            obj.Recieved_data = data;
        end
    end
end

