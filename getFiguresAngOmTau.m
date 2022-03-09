% Copyright 2017-2019 The MathWorks, Inc.

%% Setup
clc; 
close all;
robotParameters
load optimizedData_01Mar22_1220
mdlName = 'QR_Unitree';
fem_motionFront = femur_motionFront;
tib_motionFront = tibia_motionFront;
fem_motionRear = femur_motionRear;
tib_motionRear = tibia_motionRear;
%% Simulate
[q,fem_der_front,tib_der_front] = createSmoothTrajectory( ... 
    fem_motionFront,tib_motionFront,gait_period,[0 delta_gait_front]);

init_angs_F_front = [-q(1,1) -q(2,1)]; % first turn
init_angs_S_front = [-q(1,2) -q(2,2)]; % second turn


[q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
    fem_motionRear,tib_motionRear,gait_period,[0 delta_gait_rear]);

init_angs_F_rear = [-q(1,1) -q(2,1)]; % first turn
init_angs_S_rear = [-q(1,2) -q(2,2)]; % second turn

tic; simout_torque = sim(mdlName,'StopTime','10');
disp(['Compiled and ran Torque actuated simulation in ' num2str(toc) ' seconds']);
%% Torso Position Plots
figure(1)
subplot(2,1,1)
hold on
grid on
plot(get(simout_torque.yout,'measBody').Values.X.Time,get(simout_torque.yout,'measBody').Values.X.Data,'r-');
title('Robot Motion')
% legend('Torque Ctrl');
title('Simulation Output Comparisons');
xlabel('Time [s]');
ylabel('Torso X Position [m]');
subplot(2,1,2)
hold on
grid on
plot(get(simout_torque.yout,'measBody').Values.Z.Time,get(simout_torque.yout,'measBody').Values.Z.Data,'r-');
title('Robot Motion')
% legend('Torque Ctrl');
xlabel('Time [s]');
ylabel('Torso Z Position [m]');

%% Joint Plots
jointNames = {'Rfront','Rrear','Lfront','Lrear'};
for idx = 1:4
    figure(idx+1)
    jName = jointNames{idx};
    
    % Joint angle
    subplot(3,2,1)
    hold on
    grid on
    plot(get(simout_torque.yout,'measLegs').Values.(jName).AngleF.Time,get(simout_torque.yout,'measLegs').Values.(jName).AngleF.Data);
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    title([jName ' angle femur'])

    subplot(3,2,2)
    hold on
    grid on
    plot(get(simout_torque.yout,'measLegs').Values.(jName).AngleT.Time,get(simout_torque.yout,'measLegs').Values.(jName).AngleT.Data);
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    title([jName ' angle tibia'])


    % Joint speed
    subplot(3,2,3)
    hold on
    grid on
    plot(get(simout_torque.yout,'measLegs').Values.(jName).OmegaF.Time,get(simout_torque.yout,'measLegs').Values.(jName).OmegaF.Data);
    xlabel('Time [s]');
    ylabel('Speed [rad/s]');
    title([jName ' speed femur'])

    subplot(3,2,4)
    hold on
    grid on
    plot(get(simout_torque.yout,'measLegs').Values.(jName).OmegaT.Time,get(simout_torque.yout,'measLegs').Values.(jName).OmegaT.Data);
    xlabel('Time [s]');
    ylabel('Speed [rad/s]');
    title([jName ' speed tibia'])

        
    % Joint torque
    subplot(3,2,5)
    hold on
    grid on
    plot(get(simout_torque.yout,'measLegs').Values.(jName).TorqueF.Time,get(simout_torque.yout,'measLegs').Values.(jName).TorqueF.Data);
    xlabel('Time [s]');
    ylabel('Torque [N*m]');
    ylim(max_torque*[-1, 1])
    title([jName ' torque femur'])

    subplot(3,2,6)
    hold on
    grid on
    plot(get(simout_torque.yout,'measLegs').Values.(jName).TorqueT.Time,get(simout_torque.yout,'measLegs').Values.(jName).TorqueT.Data);
    xlabel('Time [s]');
    ylabel('Torque [N*m]');
    ylim(max_torque*[-1, 1])
    title([jName ' torque tibia'])
end


%% Cleanup
% bdclose(mdlName)