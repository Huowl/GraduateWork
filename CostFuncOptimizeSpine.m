function penalty = CostFuncOptimizeSpine(params,mdlName,scaleFactor,gait_period, v_x_des, actType, mean_z_des)

% Function for energy optimize spine's quadruped
% Yefim Osipov
% ITMO University

%% Init and simulate
    actuatorType = actType;
    simTime = 10;
    typeBody = 2;
    choice_ctrl_legs;
    % Load parameters into function workspace
    robotParameters;

    % Apply variable scaling
    params = scaleFactor*params;
    
    % Extract simulation inputs from parameters
    N = (numel(params)-5)/4; 
    fem_motionFront = deg2rad([params(1:N), params(1)]);
    tib_motionFront = deg2rad([params(N+1:2*N), params(N+1)]);
    fem_motionRear = deg2rad([params(2*N+1:3*N), params(2*N+1)]);
    tib_motionRear = deg2rad([params(3*N+1:4*N), params(3*N+1)]);
    delays = params(end-4:end-3);
    spine_param = params(end-2:end);
    traj_times = linspace(0,gait_period,N+1);

    
%% Create trajectory        
    % Evaluate the trajectory at the start and halfway points for right and
    % left legs, respectively, to get initial conditions and trajectory
    % waypoint derivatives
    [q,fem_der_front,tib_der_front] = createSmoothTrajectory( ... 
        fem_motionFront,tib_motionFront,gait_period,[0 delays(1)*gait_period/100]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_F_front = [q(1,1) q(2,1)]; % first turn
    init_angs_S_front = [q(1,2) q(2,2)]; % second turn

    [q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
        fem_motionRear,tib_motionRear,gait_period,[0 delays(2)*gait_period/100]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_F_rear = [q(1,1) q(2,1)]; % first turn
    init_angs_S_rear = [q(1,2) q(2,2)]; % second turn

%% Define delay gait period for couple legs

    delta_gait_front = delays(1);
    delta_gait_rear = delays(2);

%% Define spine parameters

    bd_spine_eq_pos = spine_param(1);
    bd_spine_stiffness = spine_param(2);
    bd_spine_damping = spine_param(3);

%% Simulate the model

    simout = sim(mdlName,'StopTime',num2str(simTime),'SrcWorkspace','current','FastRestart','on');          

%% Unpack logged data
    measBody = simout.measBody;
    xEnd = timeseries2timetable(measBody.x).x(end);
    tEnd = simout.tout(end);
    zData = timeseries2timetable(measBody.z).z;
    meanZ = mean(zData);

%% Aggressiveness param for more realistic movments
n = 0.2;
diffs = [diff(fem_motionFront) diff(tib_motionFront) diff(fem_motionRear) diff(tib_motionRear)];
for idx = 1:numel(diffs)-1
    if (sign(diffs(idx)/diffs(idx+1))<0) && mod(idx,N)     
           n = n + 0.8;
    end
end

aggressiveness = 2^n;

%% Penalty function

CoT = CostOfTransport(simout,actuatorType);

% Penalty velocity CoM
penVel = (1e1*(xEnd - simTime*v_x_des))^2/(tEnd)^4;

% Penalty XYZ pozition CoM
penXYZ = abs(meanZ-mean_z_des); 

penalty = prod([penVel penXYZ aggressiveness]);

% For debug
% disp(['Vel-' num2str(penVel) ' XYZ-' num2str(penXYZ) ' agr-' num2str(aggressiveness) ' pen-' num2str(penalty) 'CoT-' num2str(CoT)])

end