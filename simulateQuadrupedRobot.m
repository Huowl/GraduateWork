function penalty = simulateQuadrupedRobot(params,mdlName,scaleFactor,gait_period, v_x_des)
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
    init_angs_F_front = [-q(1,1) -q(2,1)]; % first turn
    init_angs_S_front = [-q(1,2) -q(2,2)]; % second turn

    [q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
        fem_motionRear,tib_motionRear,gait_period,[0 delays(2)*gait_period/100]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_F_rear = [-q(1,1) -q(2,1)]; % first turn
    init_angs_S_rear = [-q(1,2) -q(2,2)]; % second turn

    delta_gait_front = delays(1);
    delta_gait_rear = delays(2);

%% Simulate the model

    simout = sim(mdlName,'StopTime','10','SrcWorkspace','current','FastRestart','on');          

%% Unpack logged data
    measBody = get(simout.yout,'measBody').Values;
    yMax = max(abs(measBody.y.Data));
    zData = measBody.z.Data;
    xData = measBody.x.Data;
    xEnd = measBody.x.Data(end);
    tEnd = simout.tout(end);  

    % Velocity data
     vel_x = measBody.vX.Data;
     v_target = repmat(v_x_des,[numel(vel_x) 1]);
     timeData = simout.tout;

%     vel_z = measBody.vZ.Data;
%     omg_y = measBody.wY.Data;

%% Energy data
%     measLegs = get(simout.yout,'measLegs').Values;
%     PRfr = abs(measLegs.Rfront.OmegaT.Data'*measLegs.Rfront.TorqueT.Data) +...
%         abs(measLegs.Rfront.OmegaF.Data'*measLegs.Rfront.TorqueF.Data);
%     PRr = abs(measLegs.Rrear.OmegaT.Data'*measLegs.Rrear.TorqueT.Data) +...
%         abs(measLegs.Rrear.OmegaF.Data'*measLegs.Rrear.TorqueF.Data);
%     PLfr = abs(measLegs.Lfront.OmegaT.Data'*measLegs.Lfront.TorqueT.Data) +...
%         abs(measLegs.Lfront.OmegaF.Data'*measLegs.Lfront.TorqueF.Data);
%     PLr = abs(measLegs.Lrear.OmegaT.Data'*measLegs.Lrear.TorqueT.Data) +...
%         abs(measLegs.Lrear.OmegaF.Data'*measLegs.Lrear.TorqueF.Data);
%     Power = [PRfr PRr PLfr PLr];
%     Energy = trapz(Power(1),time) + trapz(Power(2),time) + trapz(Power(3),time) + trapz(Power(4),time);

%% OptFunction = Reward for RL from article
%     alp_1 = 0.04;
%     alp_2 = 20;

%     r_alive = 20 * v_target;
% 
%     r_energy = - (Power(1) + Power(2) + Power(3) + Power(4));

%     r_forward = -alp_2*abs(v_x_des-v_target) - vel_z.^2 - omg_y.^2;

%% First OptFunction = Optimize Travel     
%   Calculate penalty from logged data

%   Longitudinal (X) distance traveled without falling 
%   (ending simulation early) increases reward

% positiveReward = sign(xEnd)*xEnd^2 * tEnd;
    
%   Lateral (Y) displacement and trajectory aggressiveness 
%   (number of times the derivative flips signs) decreases reward
%   NOTE: Set lower limits to prevent divisions by zero

% aggressiveness = 0;
% diffs = [diff(fem_motionFront) diff(tib_motionFront) diff(fem_motionRear) diff(tib_motionRear)];
% for idx = 1:numel(diffs)-1
%     if (sign(diffs(idx)/diffs(idx+1))<0) && mod(idx,N) 
%          aggressiveness = aggressiveness + 1;            
%     end
% end
%  negativeReward = max(yMax,0.1) * max(aggressiveness,1)+ 1e-3*sum(abs(vel_x.*time - v_x_des*time));
%  negativeReward = max(yMax,0.1) * max(aggressiveness,1)+ 1e-2*sum(abs(zData - 0.3));
%  negativeReward = max(yMax,0.1) * max(aggressiveness,1);
 

%   Negative sign needed since the optimization minimizes cost function

% penalty = -positiveReward/negativeReward;
 
%% Maybe
% penalty = sqrt((vel_x - v_target)'*(vel_x - v_target)) + 3*(xEnd -
% 10*v_x_des)^2/tEnd; %1

 penalty = (xEnd - 10*v_x_des)^2/tEnd*max(yMax,0.1); %2 find walk and trot with few error
%  penalty = sqrt(sum(xData - v_x_des*timeData).^2)/tEnd*max(yMax,0.1); %2

end

