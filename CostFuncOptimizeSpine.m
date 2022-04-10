function penalty = CostFuncOptimizeSpine(params,scaleFactor,path,mdlName)

% Function for energy optimize spine's quadruped
% Yefim Osipov
% ITMO University

%% Init and simulate
    robotParameters;
    typeBody = 2;
    params = scaleFactor*params;
    full_path = ['ResultTrajectoryVersion06-04/' path];
    load(full_path);

    if exist('save_actType',"var") == 0
        actuatorType = 2;
    else
        actuatorType = save_actType;
    end
    choice_ctrl_legs;
    
    fem_motionFront = femur_motionFront;
    tib_motionFront = tibia_motionFront;
    fem_motionRear = femur_motionRear;
    tib_motionRear = tibia_motionRear;
    
    [q,fem_der_front,tib_der_front] = createSmoothTrajectory( ... 
        fem_motionFront,tib_motionFront,gait_period,[0 delta_gait_front*gait_period/100]);
    
    init_angs_F_front = [q(1,1) q(2,1)]; % first turn
    init_angs_S_front = [q(1,2) q(2,2)]; % second turn
    
    
    [q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
        fem_motionRear,tib_motionRear,gait_period,[0 delta_gait_rear*gait_period/100]);
    
    init_angs_F_rear = [q(1,1) q(2,1)]; % first turn
    init_angs_S_rear = [q(1,2) q(2,2)]; % second turn

    bd_spine_eq_pos = params(1);
    bd_spine_stiffness = params(2);
    bd_spine_damping = params(3);

    simout = sim(mdlName,'StopTime','10','SrcWorkspace','current','FastRestart','on'); 


    %% Penalty function

penalty = CostOfTransport(simout,actuatorType);

end