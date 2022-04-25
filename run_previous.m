%% Run previous
if new_run
    robotParameters; % Robot's and Ground's Parameters 

    mdlName = 'QR_Unitree';
    open_system(mdlName);
end
% Body type (with spine or not)
load(path_prev)

if exist('save_bodyType','var')
    typeBody = save_bodyType;
end
% Choose actuator type for old data 
if ~exist('save_actType',"var")
    actuatorType = 2;
else
    actuatorType = 1;%save_actType;
end
choice_ctrl_legs; % Set PID coefficient

fem_motionFront = femur_motionFront;
tib_motionFront = tibia_motionFront;
fem_motionRear = femur_motionRear;
tib_motionRear = tibia_motionRear;

% Parameters spine for second type of robot (with spine)
if save_bodyType == 2
    bd_spine_eq_pos = spine_postion;
    bd_spine_stiffness = spine_spring_stiff;
    bd_spine_damping = spine_damphing_coeff;
end

% Create Trajectory for Legs
if ~use_previous
    % For Front Legs
    [q,fem_der_front,tib_der_front] = createSmoothTrajectory( ... 
        fem_motionFront,tib_motionFront,gait_period,[0 delta_gait_front*gait_period/100]);
    
    %initial position front legs
    init_angs_F_front = [q(1,1) q(2,1)]; % first turn
    init_angs_S_front = [q(1,2) q(2,2)]; % second turn
    
    % For Rear Legs
    [q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
        fem_motionRear,tib_motionRear,gait_period,[0 delta_gait_rear*gait_period/100]);
    
    %initial position rear legs
    init_angs_F_rear = [q(1,1) q(2,1)]; % first turn
    init_angs_S_rear = [q(1,2) q(2,2)]; % second turn
    
    % Simulate
%     tic;
    prev_simout = sim(mdlName,'StopTime','10','SrcWorkspace','current');
%     simTime = toc;
end