%% 4-Way Intersection Simulation — Network Testing Version
% =========================================================
% Based on main_v2v_mpc.m (no RSU logic at all)
%
% What this file does:
%   - Runs your full intersection simulation exactly as before
%   - Cars drive using MPC/IDM and traffic light rules
%   - Right turns use gap acceptance logic (no RSU needed)
%   - RSU1 object kept ONLY so the tower appears on the plot
%   - Every car's data is sent over real UDP to PC 1 (Python)
%   - PC 1 echoes it back — we measure round-trip latency
%   - Results saved to .mat file for plot_comparison.m
%
% Run order:
%   1. PC 1: run rsu_listener.py  (Python, leave it running)
%   2. PC 2: plug in ethernet cable
%   3. PC 2: run this file → type "wired" at the end
%   4. PC 2: switch to WiFi
%   5. PC 2: run this file again → type "wifi" at the end
%   6. PC 2: run plot_comparison.m
% =========================================================

close all; clear; clc;

addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection_5g/class'))
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection_5g/function'))
addpath(genpath('C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection_5g/simplot'))

global CarN CarS CarE CarW TrafficLight
global dt KK

InitVals;
fm = 1;
dt = 0.5;
KKmax = 600;

% --- Traffic light parameters ---
Cycle = 120;
gNS = 54.5;  yNS = 3.5;  ar1 = 2.0;
gEW = 54.5;  yEW = 3.5;  ar2 = 2.0;

% --- Traffic light log ---
LightLog = struct('Time', {}, 'CycleIdx', {}, 'tInCycle', {}, ...
    'Phase', {}, 'NS', {}, 'EW', {});

% --- Intersection parameters ---
stop_line  = 10;
minGapTime = 2.0;
minGapDist = 6.0;
turn_start   = 10;
turn_wait    = 5;
turn_release = 1.5;
out_eps      = 0.5;
wait_eps     = 0.3;
appear       = 3;

% --- Car logs ---
CarLog  = struct('Time',{},'ID',{},'Dir',{},'TurnLeft',{},'TurnRight',{},'X',{},'Y',{},'V',{},'Ac',{});
CarLogN = CarLog; CarLogS = CarLog; CarLogE = CarLog; CarLogW = CarLog;

% --- Car arrays ---
CarN = Car.empty(); CarS = Car.empty();
CarE = Car.empty(); CarW = Car.empty();
CarIDN = 1000; CarIDS = 2000; CarIDE = 3000; CarIDW = 4000;

% =========================================================
% RSU — Visual only
% RSU1 is created here ONLY so SimPlotIntersection.m can
% draw the tower on the intersection map.
% It does NOT receive car data, run any logic, or send
% anything back to cars. It is purely decorative.
% =========================================================
global RSUObjs
RSU1 = RSU(1, 0, 0);
RSUObjs = [RSU1];

% =========================================================
% UDP Network Setup
%
% How it works:
%   - Each simulation step, every car packages its data
%     (ID, position, speed, direction) into a text message
%   - That message is sent over UDP to PC 1 (Python listener)
%   - PC 1 echoes the same message straight back
%   - We measure how long the round trip takes = latency
%   - If no reply comes back in UDP_TIMEOUT seconds = lost packet
%
% This lets us compare:
%   Wired ethernet  → should be fast and stable (~1-5ms)
%   WiFi            → usually slower and more variable (~5-30ms)
% =========================================================
USE_NETWORK = true;
RSU_IP      = "10.255.255.51";  % ← Replace with PC 1's IP address
RSU_PORT    = 5005;
UDP_TIMEOUT = 0.3;             % seconds to wait for echo before giving up

udp_car = udpport("datagram", "IPV4", "LocalPort", 6000);
fprintf("UDP ready → Python listener at %s:%d\n", RSU_IP, RSU_PORT);
fprintf("Make sure rsu_listener.py is running on PC 1!\n\n");

% Latency tracking
net_latency = [];  % round-trip time per packet in ms (NaN = lost)
net_lost    = 0;   % packets with no reply
net_sent    = 0;   % total packets sent

% --- Figure ---
hFig = figure;
set(hFig, 'Position', [100, 100, 1000, 800]);

% --- Traffic flow ---
spawn_gap_min  = 8;
DemandStages   = [0, 120, 300];
lambdaNS_axis  = [0.30, 0.45, 0.60];
lambdaEW_axis  = [0.25, 0.40, 0.55];

% --- Active lanes ---
CARN = true; CARS = true; CARE = true; CARW = true;

%% === Initialize first cars ===
if CARN
    carN = Car(CarIDN, -1.5, -150); CarIDN = CarIDN+1;
    carN.Dir = 'N'; CarN(end+1) = carN;
end
if CARS
    carS = Car(CarIDS, 1.5, 150); CarIDS = CarIDS+1;
    carS.Dir = 'S'; CarS(end+1) = carS;
end
if CARE
    carE = Car(CarIDE, -150, 1.5); CarIDE = CarIDE+1;
    carE.Dir = 'E'; CarE(end+1) = carE;
end
if CARW
    carW = Car(CarIDW, 150, -1.5); CarIDW = CarIDW+1;
    carW.Dir = 'W'; CarW(end+1) = carW;
end

%% ===================== MAIN LOOP =====================
for KK = 1:KKmax

    %% === Traffic Light ===
    % Cycles through NS green → NS yellow → all red → EW green → EW yellow → all red
    t    = mod(KK*dt, Cycle);
    tsec = KK * dt;

    if t < gNS
        TrafficLight.NS = 'green';  TrafficLight.EW = 'red';
    elseif t < gNS + yNS
        TrafficLight.NS = 'yellow'; TrafficLight.EW = 'red';
    elseif t < gNS + yNS + ar1
        TrafficLight.NS = 'red';    TrafficLight.EW = 'red';
    elseif t < gNS + yNS + ar1 + gEW
        TrafficLight.NS = 'red';    TrafficLight.EW = 'green';
    elseif t < gNS + yNS + ar1 + gEW + yEW
        TrafficLight.NS = 'red';    TrafficLight.EW = 'yellow';
    else
        TrafficLight.NS = 'red';    TrafficLight.EW = 'red';
    end

    cycIdx = floor((KK*dt) / Cycle) + 1;
    phs    = phaseName(t, gNS, yNS, ar1, gEW, yEW);
    LightLog(end+1) = struct('Time',KK*dt,'CycleIdx',cycIdx,'tInCycle',t, ...
        'Phase',phs,'NS',TrafficLight.NS,'EW',TrafficLight.EW);

    %% === Plot (every 2 steps) ===
    if mod(KK,2) == 1
        SimPlotIntersection;
    end

    %% === Spawn Cars (Poisson process) ===
    % Each step, there's a small random chance a new car appears
    % Probability increases over time (light → medium → heavy traffic)
    [lamN,lamS,lamE,lamW] = getApproachRates(tsec, DemandStages, lambdaNS_axis, lambdaEW_axis);
    spawnProb = @(lam) 1 - exp(-lam * dt);

    if CARN && rand < spawnProb(lamN) && canSpawnY(CarN, -150, spawn_gap_min)
        carN = Car(CarIDN,-1.5,-150); CarIDN=CarIDN+1; carN.Dir='N'; CarN(end+1)=carN;
    end
    if CARS && rand < spawnProb(lamS) && canSpawnY(CarS, 150, spawn_gap_min)
        carS = Car(CarIDS,1.5,150);   CarIDS=CarIDS+1; carS.Dir='S'; CarS(end+1)=carS;
    end
    if CARE && rand < spawnProb(lamE) && canSpawnX(CarE, -150, spawn_gap_min)
        carE = Car(CarIDE,-150,1.5);  CarIDE=CarIDE+1; carE.Dir='E'; CarE(end+1)=carE;
    end
    if CARW && rand < spawnProb(lamW) && canSpawnX(CarW, 150, spawn_gap_min)
        carW = Car(CarIDW,150,-1.5);  CarIDW=CarIDW+1; carW.Dir='W'; CarW(end+1)=carW;
    end

    %% === Compute Acceleration (MPC + Traffic Light) ===
    % Each car either follows the car ahead using MPC,
    % or stops at the stop line if the light is red/yellow.
    % A "dummy" car is used as a target when stopping at the line.

    % NORTH (cars travel in +Y direction)
    for i = 1:length(CarN)
        car = CarN(i);
        isRedOrYellow    = strcmp(TrafficLight.NS,'red') || strcmp(TrafficLight.NS,'yellow');
        isBeforeStopLine = car.Y < -stop_line;
        leadCar = []; minDist = inf;
        for j = 1:length(CarN)
            if j~=i && CarN(j).Y > car.Y
                dist = CarN(j).Y - car.Y;
                if dist < minDist, minDist=dist; leadCar=CarN(j); end
            end
        end
        if isRedOrYellow && isBeforeStopLine
            if ~isempty(leadCar) && leadCar.Y < -stop_line + car.R0
                car.Ac = car.MPC(car, leadCar);
            else
                dummy=Car(-1,0,0); dummy.Y=-stop_line+car.R0; dummy.V=0; dummy.Dir='N';
                car.Ac = car.MPC(car, dummy);
            end
        else
            if ~isempty(leadCar), car.Ac = car.MPC(car, leadCar);
            else, car.Ac = 0.5*(car.Vd - car.V); end
        end
        CarN(i) = car;
    end

    % SOUTH (cars travel in -Y direction)
    for i = 1:length(CarS)
        car = CarS(i);
        isRedOrYellow    = strcmp(TrafficLight.NS,'red') || strcmp(TrafficLight.NS,'yellow');
        isBeforeStopLine = car.Y > stop_line;
        leadCar = []; minDist = inf;
        for j = 1:length(CarS)
            if j~=i && CarS(j).Y < car.Y
                dist = car.Y - CarS(j).Y;
                if dist < minDist, minDist=dist; leadCar=CarS(j); end
            end
        end
        if isRedOrYellow && isBeforeStopLine
            if ~isempty(leadCar) && leadCar.Y > stop_line - car.R0
                car.Ac = car.MPC(car, leadCar);
            else
                dummy=Car(-1,0,0); dummy.Y=stop_line-car.R0; dummy.V=0; dummy.Dir='S';
                car.Ac = car.MPC(car, dummy);
            end
        else
            if ~isempty(leadCar), car.Ac = car.MPC(car, leadCar);
            else, car.Ac = 0.5*(car.Vd - car.V); end
        end
        CarS(i) = car;
    end

    % EAST (cars travel in +X direction)
    for i = 1:length(CarE)
        car = CarE(i);
        isRedOrYellow    = strcmp(TrafficLight.EW,'red') || strcmp(TrafficLight.EW,'yellow');
        isBeforeStopLine = car.X < -stop_line;
        leadCar = []; minDist = inf;
        for j = 1:length(CarE)
            if j~=i && CarE(j).X > car.X
                dist = CarE(j).X - car.X;
                if dist < minDist, minDist=dist; leadCar=CarE(j); end
            end
        end
        if isRedOrYellow && isBeforeStopLine
            if ~isempty(leadCar) && leadCar.X < -stop_line + car.R0
                car.Ac = car.MPC(car, leadCar);
            else
                dummy=Car(-1,0,0); dummy.X=-stop_line+car.R0; dummy.V=0; dummy.Dir='E';
                car.Ac = car.MPC(car, dummy);
            end
        else
            if ~isempty(leadCar), car.Ac = car.MPC(car, leadCar);
            else, car.Ac = 0.5*(car.Vd - car.V); end
        end
        CarE(i) = car;
    end

    % WEST (cars travel in -X direction)
    for i = 1:length(CarW)
        car = CarW(i);
        isRedOrYellow    = strcmp(TrafficLight.EW,'red') || strcmp(TrafficLight.EW,'yellow');
        isBeforeStopLine = car.X > stop_line;
        leadCar = []; minDist = inf;
        for j = 1:length(CarW)
            if j~=i && CarW(j).X < car.X
                dist = car.X - CarW(j).X;
                if dist < minDist, minDist=dist; leadCar=CarW(j); end
            end
        end
        if isRedOrYellow && isBeforeStopLine
            if ~isempty(leadCar) && leadCar.X > stop_line - car.R0
                car.Ac = car.MPC(car, leadCar);
            else
                dummy=Car(-1,0,0); dummy.X=stop_line-car.R0; dummy.V=0; dummy.Dir='W';
                car.Ac = car.MPC(car, dummy);
            end
        else
            if ~isempty(leadCar), car.Ac = car.MPC(car, leadCar);
            else, car.Ac = 0.5*(car.Vd - car.V); end
        end
        CarW(i) = car;
    end

    %% === Hard Collision Prevention ===
    % Safety override: if two cars get too close despite MPC,
    % force hard braking to prevent collision

    % East
    for i = 1:length(CarE)
        car=CarE(i); leadCar=[]; minDist=inf;
        for j=1:length(CarE)
            if j~=i && CarE(j).X>car.X
                dist=CarE(j).X-car.X;
                if dist<minDist, minDist=dist; leadCar=CarE(j); end
            end
        end
        if ~isempty(leadCar)
            gap=leadCar.X-car.X; minSafeGap=5.0;
            myNextX=car.X+car.V*dt+0.5*car.Ac*dt^2;
            leadNextX=leadCar.X+leadCar.V*dt+0.5*leadCar.Ac*dt^2;
            nextGap=leadNextX-myNextX;
            if nextGap<minSafeGap || gap<minSafeGap
                if car.V>leadCar.V
                    req=-(car.V-leadCar.V)^2/(2*max(gap-minSafeGap,0.1));
                    CarE(i).Ac=min(car.Ac,max(req,-7.0));
                end
                if gap<minSafeGap, CarE(i).Ac=-7.0;
                    fprintf('COLLISION OVERRIDE CarE %d: gap=%.2f\n',car.ID,gap); end
            end
        end
    end

    % West
    for i = 1:length(CarW)
        car=CarW(i); leadCar=[]; minDist=inf;
        for j=1:length(CarW)
            if j~=i && CarW(j).X<car.X
                dist=car.X-CarW(j).X;
                if dist<minDist, minDist=dist; leadCar=CarW(j); end
            end
        end
        if ~isempty(leadCar)
            gap=car.X-leadCar.X; minSafeGap=5.0;
            myNextX=car.X-car.V*dt-0.5*car.Ac*dt^2;
            leadNextX=leadCar.X-leadCar.V*dt-0.5*leadCar.Ac*dt^2;
            nextGap=myNextX-leadNextX;
            if nextGap<minSafeGap || gap<minSafeGap
                if car.V>leadCar.V
                    req=-(car.V-leadCar.V)^2/(2*max(gap-minSafeGap,0.1));
                    CarW(i).Ac=min(car.Ac,max(req,-7.0));
                end
                if gap<minSafeGap, CarW(i).Ac=-7.0;
                    fprintf('COLLISION OVERRIDE CarW %d: gap=%.2f\n',car.ID,gap); end
            end
        end
    end

    % North
    for i = 1:length(CarN)
        car=CarN(i); leadCar=[]; minDist=inf;
        for j=1:length(CarN)
            if j~=i && CarN(j).Y>car.Y
                dist=CarN(j).Y-car.Y;
                if dist<minDist, minDist=dist; leadCar=CarN(j); end
            end
        end
        if ~isempty(leadCar)
            gap=leadCar.Y-car.Y; minSafeGap=5.0;
            myNextY=car.Y+car.V*dt+0.5*car.Ac*dt^2;
            leadNextY=leadCar.Y+leadCar.V*dt+0.5*leadCar.Ac*dt^2;
            nextGap=leadNextY-myNextY;
            if nextGap<minSafeGap || gap<minSafeGap
                if car.V>leadCar.V
                    req=-(car.V-leadCar.V)^2/(2*max(gap-minSafeGap,0.1));
                    CarN(i).Ac=min(car.Ac,max(req,-7.0));
                end
                if gap<minSafeGap, CarN(i).Ac=-7.0;
                    fprintf('COLLISION OVERRIDE CarN %d: gap=%.2f\n',car.ID,gap); end
            end
        end
    end

    % South
    for i = 1:length(CarS)
        car=CarS(i); leadCar=[]; minDist=inf;
        for j=1:length(CarS)
            if j~=i && CarS(j).Y<car.Y
                dist=car.Y-CarS(j).Y;
                if dist<minDist, minDist=dist; leadCar=CarS(j); end
            end
        end
        if ~isempty(leadCar)
            gap=car.Y-leadCar.Y; minSafeGap=5.0;
            myNextY=car.Y-car.V*dt-0.5*car.Ac*dt^2;
            leadNextY=leadCar.Y-leadCar.V*dt-0.5*leadCar.Ac*dt^2;
            nextGap=myNextY-leadNextY;
            if nextGap<minSafeGap || gap<minSafeGap
                if car.V>leadCar.V
                    req=-(car.V-leadCar.V)^2/(2*max(gap-minSafeGap,0.1));
                    CarS(i).Ac=min(car.Ac,max(req,-7.0));
                end
                if gap<minSafeGap, CarS(i).Ac=-7.0;
                    fprintf('COLLISION OVERRIDE CarS %d: gap=%.2f\n',car.ID,gap); end
            end
        end
    end

    %% === Turn Left ===
    % Left turn logic: car slows down approaching the zone,
    % then teleports to the new direction lane when it reaches
    % the release point, if the target lane has space.

    % W -> S
    used_dummyW_left = false;
    for i = length(CarW):-1:1
        car = CarW(i);
        if car.TurnLeft==1 && car.TurnedLeft==0
            if car.X>turn_release && car.X<=turn_start
                if i>1 && CarW(i-1).TurnLeft==1 && CarW(i-1).TurnedLeft==0
                    car.Ac = car.MPC(car, CarW(i-1));
                elseif car.V>2.0 && ~used_dummyW_left
                    dummy=Car(-1,0,0); dummy.X=turn_release-car.R0; dummy.Y=car.Y; dummy.V=0;
                    car.Ac=car.MPC(car,dummy); used_dummyW_left=true;
                end
            end
            if car.X<=turn_release
                if isSpawnClear(CarS,'Y',-appear,5.0)
                    car.Dir='S'; car.X=1.5; car.Y=-appear; car.V=3;
                    car.Ac=0.5*(car.Vd-car.V); car.TurnedLeft=1;
                    idx=find([CarS.Y]>car.Y,1);
                    if isempty(idx), CarS(end+1)=car;
                    else CarS(idx+1:end+1)=CarS(idx:end); CarS(idx)=car; end
                    CarW(i)=[]; continue;
                else car.V=0; car.Ac=0; end
            end
            CarW(i)=car;
        end
    end

    % E -> N
    used_dummyE_left = false;
    for i = length(CarE):-1:1
        car = CarE(i);
        if car.TurnLeft==1 && car.TurnedLeft==0
            if car.X<-turn_release && car.X>=-turn_start
                if i>1 && CarE(i-1).TurnLeft==1 && CarE(i-1).TurnedLeft==0
                    car.Ac=car.MPC(car,CarE(i-1));
                elseif car.V>2.0 && ~used_dummyE_left
                    dummy=Car(-1,0,0); dummy.X=-turn_release+car.R0; dummy.Y=car.Y; dummy.V=0;
                    car.Ac=car.MPC(car,dummy); used_dummyE_left=true;
                end
            end
            if car.X>=-turn_release
                if isSpawnClear(CarN,'Y',appear,5.0)
                    car.Dir='N'; car.X=-1.5; car.Y=appear; car.V=3;
                    car.Ac=0.5*(car.Vd-car.V); car.TurnedLeft=1;
                    idx=find([CarN.Y]<car.Y,1);
                    if isempty(idx), CarN(end+1)=car;
                    else CarN(idx+1:end+1)=CarN(idx:end); CarN(idx)=car; end
                    CarE(i)=[]; continue;
                else car.V=0; car.Ac=0; end
            end
            CarE(i)=car;
        end
    end

    % S -> E
    used_dummyS_left = false;
    for i = length(CarS):-1:1
        car = CarS(i);
        if car.TurnLeft==1 && car.TurnedLeft==0
            if car.Y>turn_release && car.Y<=turn_start
                if i>1 && CarS(i-1).TurnLeft==1 && CarS(i-1).TurnedLeft==0
                    car.Ac=car.MPC(car,CarS(i-1));
                elseif car.V>2.0 && ~used_dummyS_left
                    dummy=Car(-1,0,0); dummy.X=car.X; dummy.Y=turn_release-car.R0; dummy.V=0;
                    car.Ac=car.MPC(car,dummy); used_dummyS_left=true;
                end
            end
            if car.Y<=turn_release
                if isSpawnClear(CarE,'X',appear,5.0)
                    car.Dir='E'; car.X=appear; car.Y=1.5; car.V=3;
                    car.Ac=0.5*(car.Vd-car.V); car.TurnedLeft=1;
                    idx=find([CarE.X]<car.X,1);
                    if isempty(idx), CarE(end+1)=car;
                    else CarE(idx+1:end+1)=CarE(idx:end); CarE(idx)=car; end
                    CarS(i)=[]; continue;
                else car.V=0; car.Ac=0; end
            end
            CarS(i)=car;
        end
    end

    % N -> W
    used_dummyN_left = false;
    for i = length(CarN):-1:1
        car = CarN(i);
        if car.TurnLeft==1 && car.TurnedLeft==0
            if car.Y<-turn_release && car.Y>=-turn_start
                if i>1 && CarN(i-1).TurnLeft==1 && CarN(i-1).TurnedLeft==0
                    car.Ac=car.MPC(car,CarN(i-1));
                elseif car.V>2.0 && ~used_dummyN_left
                    dummy=Car(-1,0,0); dummy.X=car.X; dummy.Y=-turn_release+car.R0; dummy.V=0;
                    car.Ac=car.MPC(car,dummy); used_dummyN_left=true;
                end
            end
            if car.Y>=-turn_release
                if isSpawnClear(CarW,'X',-appear,5.0)
                    car.Dir='W'; car.X=-appear; car.Y=-1.5; car.V=3;
                    car.Ac=0.5*(car.Vd-car.V); car.TurnedLeft=1;
                    idx=find([CarW.X]>car.X,1);
                    if isempty(idx), CarW(end+1)=car;
                    else CarW(idx+1:end+1)=CarW(idx:end); CarW(idx)=car; end
                    CarN(i)=[]; continue;
                else car.V=0; car.Ac=0; end
            end
            CarN(i)=car;
        end
    end

    %% === Turn Right ===
    % Right turn logic (gap acceptance — no RSU needed):
    %   1. Car slows down approaching the wait point
    %   2. At wait point: car stops and checks if it is safe to turn
    %   3. "Safe" means either:
    %      a. No conflicting cars in the oncoming lane, OR
    %      b. Oncoming cars are far enough away / slow enough, OR
    %      c. The traffic light signal allows it (not green for oncoming)
    %   4. Also checks if a left-turner has priority (yields if so)
    %   5. If safe: car teleports into the new lane and continues

    % W -> N
    used_dummyW_right = false;
    for i = length(CarW):-1:1
        car = CarW(i);
        if car.TurnRight==1 && car.TurnedRight==0
            if car.X>turn_wait && car.X<=turn_start
                if i>1 && CarW(i-1).TurnRight==1 && CarW(i-1).TurnedRight==0
                    car.Ac=car.MPC(car,CarW(i-1));
                elseif car.V>2.0 && ~used_dummyW_right
                    dummy=Car(-1,0,0); dummy.X=turn_wait-car.R0; dummy.Y=car.Y; dummy.V=0; dummy.Dir='W';
                    car.Ac=car.MPC(car,dummy); used_dummyW_right=true;
                end
            end
            if car.X<=turn_wait
                car.X=turn_wait; car.V=0; car.Ac=0;
                t_turn = estimateTurnTime(car,turn_release,turn_wait,5,-1.5);
                conf=CarE(([CarE.X]<-turn_release)&(~[CarE.TurnRight])); nconf=numel(conf);
                if nconf==0, safeToTurn=true;
                elseif nconf==1
                    other=conf(1); d=abs(-turn_release-other.X);
                    safeToTurn=(d>=minGapDist)||(d/max(other.V,0.1)>=t_turn+minGapTime);
                else
                    safeToTurn=true;
                    for j=1:nconf
                        other=conf(j); d=abs(-turn_release-other.X);
                        if (d/max(other.V,0.1)<t_turn+minGapTime)&&(d<minGapDist+2)
                            safeToTurn=false; break; end
                    end
                end
                allowOnSignal=~strcmp(TrafficLight.EW,'green');
                bothRight=leadRightAtWait(CarW,'W',turn_wait,wait_eps)&&leadRightAtWait(CarE,'E',turn_wait,wait_eps);
                leftBlocks=~bothRight&&leftTurnQueuePresent(CarE)&&leftTurnHasPriority(CarE,'E',turn_start);
                if ~leftBlocks && (safeToTurn||allowOnSignal)
                    if isSpawnClear(CarN,'Y',appear-out_eps,5.0)
                        car.Dir='N'; car.X=-1.5; car.Y=appear-out_eps; car.V=3;
                        car.Ac=0.5*(car.Vd-car.V); car.TurnedRight=1;
                        idx=find([CarN.Y]<car.Y,1);
                        if isempty(idx), CarN(end+1)=car;
                        else CarN(idx+1:end+1)=CarN(idx:end); CarN(idx)=car; end
                        CarW(i)=[]; continue;
                    end
                end
            end
            CarW(i)=car;
        end
    end

    % E -> S
    used_dummyE_right = false;
    for i = length(CarE):-1:1
        car = CarE(i);
        if car.TurnRight==1 && car.TurnedRight==0
            if car.X<-turn_wait && car.X>=-turn_start
                if i>1 && CarE(i-1).TurnRight==1 && CarE(i-1).TurnedRight==0
                    car.Ac=car.MPC(car,CarE(i-1));
                elseif car.V>2.0 && ~used_dummyE_right
                    dummy=Car(-1,0,0); dummy.X=-turn_wait+car.R0; dummy.Y=car.Y; dummy.V=0; dummy.Dir='E';
                    car.Ac=car.MPC(car,dummy); used_dummyE_right=true;
                end
            end
            if car.X>=-turn_wait
                car.X=-turn_wait; car.V=0; car.Ac=0;
                t_turn=estimateTurnTime(car,-turn_release,-turn_wait,-5,1.5);
                conf=CarW(([CarW.X]>turn_release)&(~[CarW.TurnRight])); nconf=numel(conf);
                if nconf==0, safeToTurn=true;
                elseif nconf==1
                    other=conf(1); d=abs(turn_release-other.X);
                    safeToTurn=(d>=minGapDist)||(d/max(other.V,0.1)>=t_turn+minGapTime);
                else
                    safeToTurn=true;
                    for j=1:nconf
                        other=conf(j); d=abs(turn_release-other.X);
                        if (d/max(other.V,0.1)<t_turn+minGapTime)&&(d<minGapDist+2)
                            safeToTurn=false; break; end
                    end
                end
                allowOnSignal=~strcmp(TrafficLight.EW,'green');
                bothRight=leadRightAtWait(CarE,'E',turn_wait,wait_eps)&&leadRightAtWait(CarW,'W',turn_wait,wait_eps);
                leftBlocks=~bothRight&&leftTurnQueuePresent(CarW)&&leftTurnHasPriority(CarW,'W',turn_start);
                if ~leftBlocks && (safeToTurn||allowOnSignal)
                    if isSpawnClear(CarS,'Y',-(appear-out_eps),5.0)
                        car.Dir='S'; car.X=1.5; car.Y=-(appear-out_eps); car.V=3;
                        car.Ac=0.5*(car.Vd-car.V); car.TurnedRight=1;
                        idx=find([CarS.Y]>car.Y,1);
                        if isempty(idx), CarS(end+1)=car;
                        else CarS(idx+1:end+1)=CarS(idx:end); CarS(idx)=car; end
                        CarE(i)=[]; continue;
                    end
                end
            end
            CarE(i)=car;
        end
    end

    % S -> W
    used_dummyS_right = false;
    for i = length(CarS):-1:1
        car = CarS(i);
        if car.TurnRight==1 && car.TurnedRight==0
            if car.Y>turn_wait && car.Y<=turn_start
                if i>1 && CarS(i-1).TurnRight==1 && CarS(i-1).TurnedRight==0
                    car.Ac=car.MPC(car,CarS(i-1));
                elseif car.V>2.0 && ~used_dummyS_right
                    dummy=Car(-1,0,0); dummy.Y=turn_wait-car.R0; dummy.X=car.X; dummy.V=0; dummy.Dir='S';
                    car.Ac=car.MPC(car,dummy); used_dummyS_right=true;
                end
            end
            if car.Y<=turn_wait
                car.Y=turn_wait; car.V=0; car.Ac=0;
                t_turn=estimateTurnTime(car,turn_release,turn_wait,1.5,5);
                conf=CarN(([CarN.Y]<-turn_release)&(~[CarN.TurnRight])); nconf=numel(conf);
                if nconf==0, safeToTurn=true;
                elseif nconf==1
                    other=conf(1); d=abs(-turn_release-other.Y);
                    safeToTurn=(d>=minGapDist)||(d/max(other.V,0.1)>=t_turn+minGapTime);
                else
                    safeToTurn=true;
                    for j=1:nconf
                        other=conf(j); d=abs(-turn_release-other.Y);
                        if (d/max(other.V,0.1)<t_turn+minGapTime)&&(d<minGapDist+2)
                            safeToTurn=false; break; end
                    end
                end
                allowOnSignal=~strcmp(TrafficLight.NS,'green');
                bothRight=leadRightAtWait(CarS,'S',turn_wait,wait_eps)&&leadRightAtWait(CarN,'N',turn_wait,wait_eps);
                leftBlocks=~bothRight&&leftTurnQueuePresent(CarN)&&leftTurnHasPriority(CarN,'N',turn_start);
                if ~leftBlocks && (safeToTurn||allowOnSignal)
                    if isSpawnClear(CarW,'X',-(appear-out_eps),5.0)
                        car.Dir='W'; car.X=-(appear-out_eps); car.Y=-1.5; car.V=3;
                        car.Ac=0.5*(car.Vd-car.V); car.TurnedRight=1;
                        idx=find([CarW.X]>car.X,1);
                        if isempty(idx), CarW(end+1)=car;
                        else CarW(idx+1:end+1)=CarW(idx:end); CarW(idx)=car; end
                        CarS(i)=[]; continue;
                    end
                end
            end
            CarS(i)=car;
        end
    end

    % N -> E
    used_dummyN_right = false;
    for i = length(CarN):-1:1
        car = CarN(i);
        if car.TurnRight==1 && car.TurnedRight==0
            if car.Y<-turn_wait && car.Y>=-turn_start
                if i>1 && CarN(i-1).TurnRight==1 && CarN(i-1).TurnedRight==0
                    car.Ac=car.MPC(car,CarN(i-1));
                elseif car.V>2.0 && ~used_dummyN_right
                    dummy=Car(-1,0,0); dummy.Y=-turn_wait+car.R0; dummy.X=car.X; dummy.V=0; dummy.Dir='N';
                    car.Ac=car.MPC(car,dummy); used_dummyN_right=true;
                end
            end
            if car.Y>=-turn_wait
                car.Y=-turn_wait; car.V=0; car.Ac=0;
                t_turn=estimateTurnTime(car,-turn_release,-turn_wait,-1.5,-5);
                conf=CarS(([CarS.Y]>turn_release)&(~[CarS.TurnRight])); nconf=numel(conf);
                if nconf==0, safeToTurn=true;
                elseif nconf==1
                    other=conf(1); d=abs(turn_release-other.Y);
                    safeToTurn=(d>=minGapDist)||(d/max(other.V,0.1)>=t_turn+minGapTime);
                else
                    safeToTurn=true;
                    for j=1:nconf
                        other=conf(j); d=abs(turn_release-other.Y);
                        if (d/max(other.V,0.1)<t_turn+minGapTime)&&(d<minGapDist+2)
                            safeToTurn=false; break; end
                    end
                end
                allowOnSignal=~strcmp(TrafficLight.NS,'green');
                bothRight=leadRightAtWait(CarN,'N',turn_wait,wait_eps)&&leadRightAtWait(CarS,'S',turn_wait,wait_eps);
                leftBlocks=~bothRight&&leftTurnQueuePresent(CarS)&&leftTurnHasPriority(CarS,'S',turn_start);
                if ~leftBlocks && (safeToTurn||allowOnSignal)
                    if isSpawnClear(CarE,'X',appear-out_eps,5.0)
                        car.Dir='E'; car.X=appear-out_eps; car.Y=1.5; car.V=3;
                        car.Ac=0.5*(car.Vd-car.V); car.TurnedRight=1;
                        idx=find([CarE.X]<car.X,1);
                        if isempty(idx), CarE(end+1)=car;
                        else CarE(idx+1:end+1)=CarE(idx:end); CarE(idx)=car; end
                        CarN(i)=[]; continue;
                    end
                end
            end
            CarN(i)=car;
        end
    end

    %% === Debug Print (every 10s during red) ===
    if mod(KK,20)==0 && strcmp(TrafficLight.EW,'red')
        fprintf('\n========== t=%.1fs | EW RED ==========\n', tsec);
        for i=1:min(5,length(CarW))
            c=CarW(i);
            fprintf('Car %d: X=%.2f V=%.2f Ac=%.2f %s\n', c.ID, c.X, c.V, c.Ac, ...
                ternary(c.X<-stop_line,'(BEFORE line)','(PAST line)'));
        end
        fprintf('Stop line at X=%.2f\n', -stop_line);
    end

    %% === Update Movement (position and velocity) ===
    for i=1:length(CarN), CarN(i).fdp(CarN(i)); CarN(i).V=CarN(i).fdv(CarN(i)); end
    for i=1:length(CarS), CarS(i).fdp(CarS(i)); CarS(i).V=CarS(i).fdv(CarS(i)); end
    for i=1:length(CarE), CarE(i).fdp(CarE(i)); CarE(i).V=CarE(i).fdv(CarE(i)); end
    for i=1:length(CarW), CarW(i).fdp(CarW(i)); CarW(i).V=CarW(i).fdv(CarW(i)); end

    %% === Position Clamping (last collision defence) ===
    minSafeGap = 5.0;

    for i=1:length(CarE)
        car=CarE(i);
        for j=1:length(CarE)
            if j~=i && CarE(j).X>car.X && (CarE(j).X-car.X)<minSafeGap
                CarE(i).X=CarE(j).X-minSafeGap; CarE(i).V=min(CarE(i).V,CarE(j).V);
                fprintf('CLAMPED CarE %d\n',car.ID);
            end
        end
    end
    for i=1:length(CarW)
        car=CarW(i);
        for j=1:length(CarW)
            if j~=i && CarW(j).X<car.X && (car.X-CarW(j).X)<minSafeGap
                CarW(i).X=CarW(j).X+minSafeGap; CarW(i).V=min(CarW(i).V,CarW(j).V);
                fprintf('CLAMPED CarW %d\n',car.ID);
            end
        end
    end
    for i=1:length(CarN)
        car=CarN(i);
        for j=1:length(CarN)
            if j~=i && CarN(j).Y>car.Y && (CarN(j).Y-car.Y)<minSafeGap
                CarN(i).Y=CarN(j).Y-minSafeGap; CarN(i).V=min(CarN(i).V,CarN(j).V);
                fprintf('CLAMPED CarN %d\n',car.ID);
            end
        end
    end
    for i=1:length(CarS)
        car=CarS(i);
        for j=1:length(CarS)
            if j~=i && CarS(j).Y<car.Y && (car.Y-CarS(j).Y)<minSafeGap
                CarS(i).Y=CarS(j).Y+minSafeGap; CarS(i).V=min(CarS(i).V,CarS(j).V);
                fprintf('CLAMPED CarS %d\n',car.ID);
            end
        end
    end

    %% === V2X over UDP Network ===
    % This is where the real network testing happens.
    %
    % Every simulation step, we loop through ALL active cars.
    % For each car we:
    %   1. Build a text message with the car's current state
    %   2. Send it as a UDP packet to PC 1 (Python listener)
    %   3. Start a stopwatch
    %   4. Wait up to UDP_TIMEOUT seconds for PC 1 to echo it back
    %   5. Record the round-trip time as latency
    %   6. If no reply: record as lost packet (NaN)
    %
    % The message format is plain text, pipe-separated:
    %   V2X | CarID | SimTime | X | Y | Speed | Accel | Direction | TurnLeft | TurnRight
    %
    % PC 1 (rsu_listener.py) reads this, prints it, and sends it back.
    % No processing or decisions happen on PC 1 — it is purely a network test.

    if USE_NETWORK
        allCarsNet = [CarN, CarS, CarE, CarW];

        for i = 1:length(allCarsNet)
            car = allCarsNet(i);

            msg = sprintf("V2X|%d|%.2f|%.3f|%.3f|%.3f|%.4f|%s|%d|%d", ...
                car.ID, tsec, car.X, car.Y, car.V, car.Ac, ...
                car.Dir, car.TurnLeft, car.TurnRight);

            % Send packet
            t_send = tic;
            write(udp_car, uint8(char(msg)), "uint8", RSU_IP, RSU_PORT);
            net_sent = net_sent + 1;

            % Wait a moment for the echo to physically travel back
            pause(0.01);  % 10ms grace period

            % Now check for reply
            got_ack  = false;
            deadline = tic;
            while toc(deadline) < UDP_TIMEOUT
                if udp_car.NumDatagramsAvailable > 0
                    read(udp_car, 1, "uint8");
                    net_latency(end+1) = toc(t_send) * 1000;
                    fprintf("Car %d → %.2f ms\n", car.ID, net_latency(end));
                    got_ack = true;
                    break;
                end
                pause(0.001);  % small poll interval
            end

            if ~got_ack
                net_latency(end+1) = NaN;
                net_lost = net_lost + 1;
            end
        end
    end

    %% === Log Data ===
    for i=1:length(CarN)
        CarLog(end+1)=struct('Time',KK*dt,'ID',CarN(i).ID,'Dir',CarN(i).Dir,...
            'TurnLeft',CarN(i).TurnLeft,'TurnRight',CarN(i).TurnRight,...
            'X',CarN(i).X,'Y',CarN(i).Y,'V',CarN(i).V,'Ac',CarN(i).Ac);
        CarLogN(end+1)=CarLog(end);
    end
    for i=1:length(CarS)
        CarLog(end+1)=struct('Time',KK*dt,'ID',CarS(i).ID,'Dir',CarS(i).Dir,...
            'TurnLeft',CarS(i).TurnLeft,'TurnRight',CarS(i).TurnRight,...
            'X',CarS(i).X,'Y',CarS(i).Y,'V',CarS(i).V,'Ac',CarS(i).Ac);
        CarLogS(end+1)=CarLog(end);
    end
    for i=1:length(CarE)
        CarLog(end+1)=struct('Time',KK*dt,'ID',CarE(i).ID,'Dir',CarE(i).Dir,...
            'TurnLeft',CarE(i).TurnLeft,'TurnRight',CarE(i).TurnRight,...
            'X',CarE(i).X,'Y',CarE(i).Y,'V',CarE(i).V,'Ac',CarE(i).Ac);
        CarLogE(end+1)=CarLog(end);
    end
    for i=1:length(CarW)
        CarLog(end+1)=struct('Time',KK*dt,'ID',CarW(i).ID,'Dir',CarW(i).Dir,...
            'TurnLeft',CarW(i).TurnLeft,'TurnRight',CarW(i).TurnRight,...
            'X',CarW(i).X,'Y',CarW(i).Y,'V',CarW(i).V,'Ac',CarW(i).Ac);
        CarLogW(end+1)=CarLog(end);
    end

    %% === Remove Out of Bounds Cars ===
    CarN = CarN(abs([CarN.X])<=170 & abs([CarN.Y])<=170);
    CarS = CarS(abs([CarS.X])<=170 & abs([CarS.Y])<=170);
    CarE = CarE(abs([CarE.X])<=170 & abs([CarE.Y])<=170);
    CarW = CarW(abs([CarW.X])<=170 & abs([CarW.Y])<=170);

end % end main loop

%% === Network Results ===
if USE_NETWORK && net_sent > 0
    valid_lat = net_latency(~isnan(net_latency));
    loss_pct  = (net_lost / net_sent) * 100;

    % Compute all stats
    lat_mean = mean(valid_lat);
    lat_min  = min(valid_lat);
    lat_max  = max(valid_lat);
    lat_std  = std(valid_lat);
    lat_med  = median(valid_lat);
    lat_p95  = prctile(valid_lat, 95);

    % Print table
    fprintf('\n');
    fprintf('╔══════════════════════════════════════╗\n');
    fprintf('║       NETWORK PERFORMANCE RESULTS    ║\n');
    fprintf('╠══════════════════════════════════════╣\n');
    fprintf('║ Packets Sent       : %10d     ║\n', net_sent);
    fprintf('║ Packets Lost       : %10d     ║\n', net_lost);
    fprintf('║ Packet Loss        : %9.2f%%     ║\n', loss_pct);
    fprintf('╠══════════════════════════════════════╣\n');
    fprintf('║ Mean Latency       : %8.2f ms     ║\n', lat_mean);
    fprintf('║ Min Latency        : %8.2f ms     ║\n', lat_min);
    fprintf('║ Max Latency        : %8.2f ms     ║\n', lat_max);
    fprintf('║ Median Latency     : %8.2f ms     ║\n', lat_med);
    fprintf('║ Std Deviation      : %8.2f ms     ║\n', lat_std);
    fprintf('║ 95th Percentile    : %8.2f ms     ║\n', lat_p95);
    fprintf('╚══════════════════════════════════════╝\n\n');

    % Save to .mat file
    mode     = input("Save as (wired / wifi): ", "s");
    filename = sprintf("results_%s.mat", mode);
    save(filename, "net_latency", "net_lost", "net_sent", ...
        "valid_lat",  "loss_pct", "mode", ...
        "lat_mean", "lat_min", "lat_max", ...
        "lat_med",  "lat_std", "lat_p95");
    fprintf("Saved to: %s\n", filename);
    fprintf("Run plot_comparison.m after both wired and wifi tests.\n");
end

%% === Trajectory Plots ===
plot_EW_trajectories(LightLog, CarLogE, CarLogW, stop_line, ...
    'C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection_5g/output/EW_network.png');
plot_NS_trajectories(LightLog, CarLogS, CarLogN, stop_line, ...
    'C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection_5g/output/NS_network.png');
plot_EW_accelerations(LightLog, CarLogS, CarLogN, stop_line, ...
    'C:/Users/monea/OneDrive/Documents/MATLAB/Traffic_Intersection_5g/output/EW_network_ac.png');


%% === Helper Functions ===

function s = ternary(cond, a, b)
if cond, s=a; else, s=b; end
end

function t_turn = estimateTurnTime(car, turn_release, turn_wait, turn_waitX, turn_waitY)
d_entry   = sqrt((car.X-turn_waitX)^2 + (car.Y-turn_waitY)^2);
flat      = (turn_wait - turn_release) / 2;
arc       = sqrt(2 * flat^2);
total_dist = d_entry + 2*flat + arc;
v_turn    = min((car.V + car.Vd)/2, 3);
t_turn    = total_dist / max(v_turn, 0.1);
end

function [lamN,lamS,lamE,lamW] = getApproachRates(tsec, stages, lamNS_axis, lamEW_axis)
if tsec<stages(2), k=1; elseif tsec<stages(3), k=2; else, k=3; end
lamNS=lamNS_axis(k); lamEW=lamEW_axis(k);
lamN=0.5*lamNS; lamS=0.5*lamNS; lamE=0.5*lamEW; lamW=0.5*lamEW;
end

function ok = canSpawnY(list, y0, gap)
if isempty(list), ok=true; return; end
ok = all(abs([list.Y]-y0) >= gap);
end

function ok = canSpawnX(list, x0, gap)
if isempty(list), ok=true; return; end
ok = all(abs([list.X]-x0) >= gap);
end

function hasLeftQueue = leftTurnQueuePresent(list)
if isempty(list), hasLeftQueue=false; return; end
hasLeftQueue = any(([list.TurnLeft]==1) & (~[list.TurnedLeft]));
end

function waitLeft = leftTurnHasPriority(opList, oppDir, prioWin)
waitLeft=false; if isempty(opList), return; end
switch oppDir
    case 'E', cand=([opList.TurnLeft]==1)&(~[opList.TurnedLeft])&([opList.X]>=-prioWin);
    case 'W', cand=([opList.TurnLeft]==1)&(~[opList.TurnedLeft])&([opList.X]<= prioWin);
    case 'N', cand=([opList.TurnLeft]==1)&(~[opList.TurnedLeft])&([opList.Y]>=-prioWin);
    case 'S', cand=([opList.TurnLeft]==1)&(~[opList.TurnedLeft])&([opList.Y]<= prioWin);
    otherwise, cand=false(size(opList));
end
waitLeft=any(cand);
end

function yes = leadRightAtWait(list, dir, turn_wait, eps)
yes=false; if isempty(list), return; end
switch dir
    case 'E', [~,k]=max([list.X]);  c=list(k); yes=c.TurnRight&&~c.TurnedRight&&abs(c.X+turn_wait)<=eps;
    case 'W', [~,k]=min([list.X]);  c=list(k); yes=c.TurnRight&&~c.TurnedRight&&abs(c.X-turn_wait)<=eps;
    case 'N', [~,k]=max([list.Y]);  c=list(k); yes=c.TurnRight&&~c.TurnedRight&&abs(c.Y+turn_wait)<=eps;
    case 'S', [~,k]=min([list.Y]);  c=list(k); yes=c.TurnRight&&~c.TurnedRight&&abs(c.Y-turn_wait)<=eps;
    otherwise, yes=false;
end
end

function phase = phaseName(t, gNS, yNS, ar1, gEW, yEW)
if t<gNS, phase='NS_GREEN';
elseif t<gNS+yNS, phase='NS_YELLOW';
elseif t<gNS+yNS+ar1, phase='ALL_RED_1';
elseif t<gNS+yNS+ar1+gEW, phase='EW_GREEN';
elseif t<gNS+yNS+ar1+yEW, phase='EW_YELLOW';
else, phase='ALL_RED_2'; end
end

function clear = isSpawnClear(carList, axis, spawnPos, safeGap)
clear=true; if isempty(carList), return; end
if axis=='X', positions=[carList.X]; else, positions=[carList.Y]; end
if any(abs(positions-spawnPos)<safeGap), clear=false; end
end