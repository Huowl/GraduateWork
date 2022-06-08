function penalty = simulateQuadrupedRobot(params,mdlName,scaleFactor,gait_period, v_x_des, actType, mean_z_des)
% Cost function for robot quadruped optimization
% Ivan Borisov
% ITMO University
    actuatorType = actType;
    simTime = 10;
    typeBody = 1;
    choice_ctrl_legs;
    % Load parameters into function workspace
    robotParameters;
    
    bd_spine_eq_pos = 0;
    bd_spine_stiffness = 0;
    bd_spine_damping = 0;

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
    init_angs_F_front = [q(1,1) q(2,1)]; % first turn deg2rad([-140 95]);%
    init_angs_S_front = [q(1,2) q(2,2)]; % second turn

    [q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
        fem_motionRear,tib_motionRear,gait_period,[0 delays(2)*gait_period/100]);
    % Package up the initial conditions, keeping the yaw/roll joints fixed
    init_angs_F_rear = [q(1,1) q(2,1)]; % first turn
    init_angs_S_rear = [q(1,2) q(2,2)]; % second turn

    delta_gait_front = delays(1);
    delta_gait_rear = delays(2);

%% Simulate the model

    simout = sim(mdlName,'StopTime',num2str(simTime),'SrcWorkspace','current','FastRestart','off');          

%% Unpack logged data
    measBody = simout.measBody;
    xEnd = timeseries2timetable(measBody.x).x(end);
    tEnd = simout.tout(end);
    zData = timeseries2timetable(measBody.z).z;
% 
     vel_x = timeseries2timetable(measBody.vX).vX;
% %      v_target = repmat(v_x_des,[numel(vel_x) 1]);
% 
    yData = timeseries2timetable(measBody.y).y;
    xData = timeseries2timetable(measBody.x).x;

%% Aggressiveness param for more realistic movments
n = 0;
diffs = [diff(fem_motionFront) diff(tib_motionFront) diff(fem_motionRear) diff(tib_motionRear)];
for idx = 1:numel(diffs)-1
    if (sign(diffs(idx)/diffs(idx+1))<0) && mod(idx,N)     
           n = n + 1;
    end
end

aggressiveness = 2^n;
%% Distance 

delta_x = diff(xData);
delta_y = diff(yData);
delta_z = diff(zData);
del_radius_vector = sqrt(delta_x.^2 + delta_y.^2 + delta_z.^2);
distance = sum(del_radius_vector);

%% Route

s_x = xData(end) - xData(1);
s_y = yData(end) - yData(1);
s_z = zData(end) - zData(1);

route = sqrt(s_x^2+s_y^2+s_z^2);

%% fitness function

CoT = (5e1*CostOfTransport(simout,actuatorType))^2;

% Penalty velocity CoM
% penVel = (10*(vel_x - v_target))'*(10*(vel_x - v_target))/(tEnd)^4;
penVel = (1e2*(xEnd - simTime*v_x_des))^2/(tEnd)^4;

% Penalty XYZ pozition CoM
% penXYZ = abs(meanZ-mean_z_des);
% penXYZ = distance/route; 

penalty = prod([max(penVel,5e-3) CoT]);

% For debug
% disp([num2str(mean(vel_x)) ' Vel-' num2str(penVel)  ' CoT-' num2str(sqrt(CoT)/50) ' agr-' num2str(aggressiveness) ' pen-' num2str(penalty)])
end

