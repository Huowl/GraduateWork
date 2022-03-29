function penalty = CostFuncOptimizeSpine(params,path,mdlName)
% Function for energy optimize spine's quadruped
% Yefim Osipov
% ITMO University
%% Init and simulate
    robotParameters;
    full_path = ['ResultOptimizeTrajectory/' path];
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
    
    init_angs_F_front = [-q(1,1) -q(2,1)]; % first turn
    init_angs_S_front = [-q(1,2) -q(2,2)]; % second turn
    
    
    [q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
        fem_motionRear,tib_motionRear,gait_period,[0 delta_gait_rear*gait_period/100]);
    
    init_angs_F_rear = [-q(1,1) -q(2,1)]; % first turn
    init_angs_S_rear = [-q(1,2) -q(2,2)]; % second turn

    spine_eq_pos = params(1);
    spine_stiffness = params(2);
    spine_damping = params(3);

    simout = sim(mdlName,'StopTime','10','SrcWorkspace','current','FastRestart','on'); 

    %% Energy data
    measLegs = get(simout.yout,'measLegs').Values;
    if actuatorType == 2 % If PD ctrl
        PRfr = abs(measLegs.Rfront.OmegaT.Data'*measLegs.Rfront.TorqueT.Data) +...
            abs(measLegs.Rfront.OmegaF.Data'*measLegs.Rfront.TorqueF.Data);
        PRr = abs(measLegs.Rrear.OmegaT.Data'*measLegs.Rrear.TorqueT.Data) +...
            abs(measLegs.Rrear.OmegaF.Data'*measLegs.Rrear.TorqueF.Data);
        PLfr = abs(measLegs.Lfront.OmegaT.Data'*measLegs.Lfront.TorqueT.Data) +...
            abs(measLegs.Lfront.OmegaF.Data'*measLegs.Lfront.TorqueF.Data);
        PLr = abs(measLegs.Lrear.OmegaT.Data'*measLegs.Lrear.TorqueT.Data) +...
            abs(measLegs.Lrear.OmegaF.Data'*measLegs.Lrear.TorqueF.Data);
        Power = [PRfr PRr PLfr PLr];
        Energy = trapz(Power(1),time) + trapz(Power(2),time) + trapz(Power(3),time) + trapz(Power(4),time);
    elseif actuatorType == 1 % If Motor
        measPow = get(simout.yout,'PowLegs').Values;
        P_front_right = abs(measPow.PwRfront.VoltageFemur.Data.*measPow.PwRfront.CurrentFemur.Data) ...
            + abs(measPow.PwRfront.VoltageTibia.Data.*measPow.PwRfront.CurrentTibia.Data);
        P_front_left = abs(measPow.PwLfront.VoltageFemur.Data.*measPow.PwLfront.CurrentFemur.Data) ...
            + abs(measPow.PwLfront.VoltageTibia.Data.*measPow.PwLfront.CurrentTibia.Data);
        P_rear_right = abs(measPow.PwRrear.VoltageFemur.Data.*measPow.PwRrear.CurrentFemur.Data) ... 
            + abs(measPow.PwRrear.VoltageTibia.Data.* measPow.PwRrear.CurrentTibia.Data);
        P_rear_left = abs(measPow.PwLrear.VoltageFemur.Data.*measPow.PwLrear.CurrentFemur.Data) ... 
            + abs(measPow.PwLrear.VoltageTibia.Data.*measPow.PwLrear.CurrentTibia.Data);
        Energy = trapz(timeData,P_front_left) + trapz(timeData,P_front_right)...
            + trapz(timeData,P_rear_left) + trapz(timeData,P_rear_right); 
    end

    %% Penalty function

    penalty = Energy; %Change later
end