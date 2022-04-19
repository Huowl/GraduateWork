%% Run previous

robotParameters; % Robot's and Ground's Parameters 

mdlName = 'QR_Unitree';
open_system(mdlName);

% Body type (with spine or not)
load(path_prev)

% Choose actuator type for old data 
if exist('save_actType',"var") == 0
    actuatorType = 2;
else
    actuatorType = save_actType;
end
choice_ctrl_legs; % Set PID coefficient

fem_motionFront = femur_motionFront;
tib_motionFront = tibia_motionFront;
fem_motionRear = femur_motionRear;
tib_motionRear = tibia_motionRear;

% Create Trajectory for Legs

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
tic;
prev_simout = sim(mdlName,'StopTime','10','SrcWorkspace','current');
simTime = toc;