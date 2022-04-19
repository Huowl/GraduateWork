function penalty = simulateQuadrupedRobot(params,mdlName,scaleFactor,gait_period, v_x_des, actType,mean_z_des)
% Cost function for robot quadruped optimization
% Ivan Borisov
% ITMO University
    actuatorType = actType;
    typeBody = 1;
    simTime = 10;
    choice_ctrl_legs;
    % Load parameters into function workspace
    robotParameters;
    % Trajectory sample time
    tsTraj = 0.01;          
    % Simulate air drag for stability 
    world_damping = 20;     % Translational damping
    world_rot_damping = 10; % Rotational damping
    
    % Apply variable scaling
    params = scaleFactor*params;
    
    % Extract simulation inputs from parameters
    N = (numel(params)-2)/4; 
    fem_motionFront = deg2rad([params(1:N), params(1)]);
    tib_motionFront = deg2rad([params(N+1:2*N), params(N+1)]);
    fem_motionRear = deg2rad([params(2*N+1:3*N), params(2*N+1)]);
    tib_motionRear = deg2rad([params(3*N+1:4*N), params(3*N+1)]);
    delays = params(end-1:end);
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

    delta_gait_front = delays(1);
    delta_gait_rear = delays(2);

%% Simulate the model

    simout = sim(mdlName,'StopTime',num2str(simTime),'SrcWorkspace','current','FastRestart','on');          

%% Unpack logged data
    measBody = simout.measBody;
    xEnd = timeseries2timetable(measBody.x).x(end);
    tEnd = simout.tout(end);
    zData = timeseries2timetable(measBody.x).x;
    meanZ = mean(zData);
%     velData = timeseries2timetable(measBody.vX).vX;
%     v_target = repmat(v_x_des,[numel(velData) 1]);

%% Aggressiveness param for more realistic movment

aggressiveness = 0;
diffs = [diff(fem_motionFront) diff(tib_motionFront) diff(fem_motionRear) diff(tib_motionRear)];
for idx = 1:numel(diffs)-1
    if (sign(diffs(idx)/diffs(idx+1))<0) && mod(idx,N) 
         aggressiveness = aggressiveness + 1;            
    end
end

%% fitness function

CoT = CostOfTransport(simout,actuatorType);

% Penalty velocity CoM
penVel = (10*(xEnd - simTime*v_x_des))^2/(tEnd/2)^2;
% Penalty Z pozition CoM
penZ = abs(meanZ-mean_z_des);

penalty = zeros(1,2);
penalty(1) = penVel*penZ*(aggressiveness+1);
penalty(2) = CoT;

end

