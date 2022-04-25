%% Testing Cost Function

% path = ['ResultOptimize18-04/' 'walk_PD_ctrl_optimizedData_19Apr22_1152'];

% mdlName = 'QR_Unitree';
% open_system(mdlName);
% load(path)

% Choose actuator type for old data 
% if exist('save_actType',"var") == 0
%     actuatorType = 2;
% else
%     actuatorType = save_actType;
% end
% choice_ctrl_legs; % Set PID coefficient
% 
% fem_motionFront = femur_motionFront;
% tib_motionFront = tibia_motionFront;
% fem_motionRear = femur_motionRear;
% tib_motionRear = tibia_motionRear;

% Set desired velocity for optimization
% v_des = 1.5; % [m/s]

% Set desired mean z position
% z_des = 0.4; % [m]
% scalingFactor = 3;
% set_param(mdlName,'SimulationMode','accelerator');
% set_param(mdlName,'SimMechanicsOpenEditorOnUpdate','on');
% robotParameters;
last_scores(1,2) = 0;
% numPoints = 6; % Number of joint angle points
for idx = 1:size(last_scores)
p = last_population(idx,:);
% p = [rad2deg(femur_motionFront(1:end-1)),...
%     rad2deg(tibia_motionFront(1:end-1)),...
%     rad2deg(femur_motionRear(1:end-1)),...
%     rad2deg(tibia_motionRear(1:end-1)),...
%     delta_gait_front, delta_gait_rear, spine_position, spine_spring_stiff, spine_damphing_coeff]/scalingFactor;
disp(['indiv: ' num2str(idx)]);
% costFcn = simulateQuadrupedRobot(p,mdlName,scalingFactor,gait_period,v_des,actuatorType,z_des);
costFcn = CostFuncOptimizeSpine(p,mdlName,scalingFactor,gait_period, v_des, actuatorType, 0);
last_scores(idx,2) = costFcn;
end
% disp(costFcn);
% set_param(mdlName,'FastRestart','off'); % Off Fast Restart