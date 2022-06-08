function penalty = simulateQuadrupedRobot(params,mdlName,scaleFactor,file_path)
% Cost function for robot quadruped optimization
% Yefim Osipov
% ITMO University
    actuatorType = 1;
    simTime = 10;
    typeBody = 1;
    % Load parameters into function workspace
    robotParameters;
    addpath(genpath('ResultOptimize18-04'))
    addpath(genpath('ResultOptimRigid27_04'))
    load(file_path);

    fem_motionFront = femur_motionFront;
    tib_motionFront = tibia_motionFront;
    fem_motionRear = femur_motionRear;
    tib_motionRear = tibia_motionRear;
    
    bd_spine_eq_pos = 0;
    bd_spine_stiffness = 0;
    bd_spine_damping = 0;

    % Apply variable scaling
    params = scaleFactor*params;


    k_p_fem =  params(1);
    k_d_fem =  params(2)/10;
    k_i_fem = 0;
    k_p_tib =  params(1);
    k_d_tib =  params(2)/10;
    k_i_tib = 0;
%% Create trajectory        

    [q,fem_der_front,tib_der_front] = createSmoothTrajectory( ... 
        fem_motionFront,tib_motionFront,gait_period,[0 delta_gait_front*gait_period/100]);
    
    init_angs_F_front = deg2rad([-140 95]);%[q(1,1) q(2,1)]; % first turn
    init_angs_S_front = deg2rad([-140 95]);%[q(1,2) q(2,2)]; % second turn
    
    
    [q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
        fem_motionRear,tib_motionRear,gait_period,[0 delta_gait_rear*gait_period/100]);
    
    init_angs_F_rear = deg2rad([-140 95]);%[q(1,1) q(2,1)]; % first turn
    init_angs_S_rear = deg2rad([-140 95]);%[q(1,2) q(2,2)]; % second turn

%% Simulate the model

    simout = sim(mdlName,'StopTime',num2str(simTime),'SrcWorkspace','current','FastRestart','off');          

%% Unpack logged data
    measBody = simout.measBody;
    xEnd = timeseries2timetable(measBody.x).x(end);
    tEnd = simout.tout(end);
    zData = timeseries2timetable(measBody.z).z;
% 
     vel_x = timeseries2timetable(measBody.vX).vX;
%      v_target = repmat(v_x_des,[numel(vel_x) 1]);
% 
    yData = timeseries2timetable(measBody.y).y;
    xData = timeseries2timetable(measBody.x).x;

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

    CoT = (1e1*CostOfTransport(simout,actuatorType))^-2;
    penVel = (2e1*vel_x'*vel_x)*(tEnd)^4;
    penalty = -prod([penVel CoT]);

% For debug
% disp([num2str(mean(vel_x)) ' Vel-' num2str(penVel)  ' CoT-' num2str(sqrt(CoT)) ' agr-' num2str(aggressiveness) ' pen-' num2str(penalty)])
end

