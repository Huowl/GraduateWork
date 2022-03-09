function penalty = simulateQuadrupedRobot(params,mdlName,scaleFactor,gait_period,delta_gait_front,delta_gait_rear)
% Cost function for robot quadruped optimization
% Ivan Borisov
% ITMO University

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
    N = numel(params)/4; 
    fem_motionFront = deg2rad([params(1:N), params(1)]);
    tib_motionFront = deg2rad([params(N+1:2*N), params(N+1)]);
    fem_motionRear = deg2rad([params(2*N+1:3*N), params(2*N+1)]);
    tib_motionRear = deg2rad([params(3*N+1:4*N), params(3*N+1)]);
    traj_times = linspace(0,gait_period,N+1);

        
    % Evaluate the trajectory at the start and halfway points for right and
    % left legs, respectively, to get initial conditions and trajectory
    % waypoint derivatives
    [q,fem_der_front,tib_der_front] = createSmoothTrajectory( ... 
        fem_motionFront,tib_motionFront,gait_period,[0 delta_gait_front]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_F_front = [-q(1,1) -q(2,1)]; % first turn
    init_angs_S_front = [-q(1,2) -q(2,2)]; % second turn

    [q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
        fem_motionRear,tib_motionRear,gait_period,[0 delta_gait_rear]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_F_rear = [-q(1,1) -q(2,1)]; % first turn
    init_angs_S_rear = [-q(1,2) -q(2,2)]; % second turn

    % Simulate the model
    simout = sim(mdlName,'StopTime','10','SrcWorkspace','current','FastRestart','on');          

    % Unpack logged data
%     measBody = get(simout.simout,'measBody').Values;
    measBody = get(simout.yout,'measBody').Values;
    yMax = max(abs(measBody.X.Data));
    xEnd = measBody.X.Data(end);
    tEnd = simout.tout(end);

    % Calculate penalty from logged data
    
    %   Longitudinal (Y) distance traveled without falling 
    %   (ending simulation early) increases reward
    positiveReward = sign(xEnd)*xEnd^2 * tEnd;
    
    %   Lateral (Y) displacement and trajectory aggressiveness 
    %   (number of times the derivative flips signs) decreases reward
    %   NOTE: Set lower limits to prevent divisions by zero
    aggressiveness = 0;
    diffs = [diff(fem_motionFront) diff(tib_motionFront) diff(fem_motionRear) diff(tib_motionRear)];
    for idx = 1:numel(diffs)-1
        if (sign(diffs(idx)/diffs(idx+1))<0) && mod(idx,N) 
             aggressiveness = aggressiveness + 1;            
        end
    end
    negativeReward = max(yMax,0.1) * max(aggressiveness,1);
    
    %   Negative sign needed since the optimization minimizes cost function     
    penalty = -positiveReward/negativeReward;        
    
end

