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
    N = numel(params)/2; 
    fem_motion = deg2rad([params(1:N), params(1)]);
    tib_motion = deg2rad([params(N+1:2*N), params(N+1)]);
    traj_times = linspace(0,gait_period,N+1);

        
    % Evaluate the trajectory at the start and halfway points for right and
    % left legs, respectively, to get initial conditions and trajectory
    % waypoint derivatives
    [q,fem_der,tib_der] = createSmoothTrajectory( ... 
        fem_motion,tib_motion,gait_period,[0 gait_period/2]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_F = [-q(1,1) -q(2,1)]; % first turn
    init_angs_S = [-q(1,2) -q(2,2)]; % second turn

    % Simulate the model
    simout = sim(mdlName,'StopTime','10','SrcWorkspace','current','FastRestart','on');          

    % Unpack logged data
    measBody = get(simout.simout,'measBody').Values;
    xMax = max(abs(measBody.X.Data));
    yEnd = measBody.Y.Data(end);
    tEnd = simout.tout(end);

    % Calculate penalty from logged data
    
    %   Longitudinal (Y) distance traveled without falling 
    %   (ending simulation early) increases reward
    positiveReward = sign(yEnd)*yEnd^2 * tEnd;
    
    %   Lateral (Y) displacement and trajectory aggressiveness 
    %   (number of times the derivative flips signs) decreases reward
    %   NOTE: Set lower limits to prevent divisions by zero
    aggressiveness = 0;
    diffs = [diff(fem_motion) diff(tib_motion)];
    for idx = 1:numel(diffs)-1
        if (sign(diffs(idx)/diffs(idx+1))<0) && mod(idx,N) 
             aggressiveness = aggressiveness + 1;            
        end
    end
    negativeReward = max(xMax,0.1) * max(aggressiveness,1);
    
    %   Negative sign needed since the optimization minimizes cost function     
    penalty = -positiveReward/negativeReward;   