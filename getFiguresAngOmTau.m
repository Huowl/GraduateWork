% Copyright 2017-2019 The MathWorks, Inc.

%% Setup
clc; 
close all;
robotParameters;
mdlName = 'QR_Unitree';
typeBody = 2;
addpath(genpath('ResultOptimizeTrajectory'));
addpath(genpath('ResultTrajectoryVersion06-04'));
load bouncing_num_11_PD_ctrl_optimizedData_06Apr22_1548
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
%% Simulate
[q,fem_der_front,tib_der_front] = createSmoothTrajectory( ... 
    fem_motionFront,tib_motionFront,gait_period,[0 delta_gait_front*gait_period/100]);

init_angs_F_front = [q(1,1) q(2,1)]; % first turn
init_angs_S_front = [q(1,2) q(2,2)]; % second turn


[q,fem_der_rear,tib_der_rear] = createSmoothTrajectory(... 
    fem_motionRear,tib_motionRear,gait_period,[0 delta_gait_rear*gait_period/100]);

init_angs_F_rear = [q(1,1) q(2,1)]; % first turn
init_angs_S_rear = [q(1,2) q(2,2)]; % second turn

tic; simout_torque = sim(mdlName,'StopTime','10');
disp(['Compiled and ran Torque actuated simulation in ' num2str(toc) ' seconds ode23s']);
%% Torso Position Plots
mean_z_pos = sum(simout_torque.measBody.z.Data)/numel(simout_torque.measBody.z.Data);

size_arr_vel = numel(simout_torque.measBody.vX.Data);
mean_vel = sum(simout_torque.measBody.vX.Data)/numel(simout_torque.measBody.vX.Data);
arr_mean_vel = repmat(mean_vel,[1 size_arr_vel]);

x_data = simout_torque.measBody.x.Data;
size_x = numel(x_data);
des_x = linspace(0,simout_torque.measBody.x.Time(end)*1.5,size_x);

CoT = CostOfTransport(simout_torque,actuatorType);

figure(1)
subplot(3,1,1)
hold on
grid on
plot(simout_torque.measBody.x.Time,simout_torque.measBody.x.Data,simout_torque.measBody.x.Time,des_x);
title('Robot Motion')
% legend('Torque Ctrl');
title('Simulation Output Comparisons');
xlabel('Time [s]');
ylabel('X Position [m]');
subplot(3,1,2)
hold on
grid on
plot(simout_torque.measBody.z.Time,simout_torque.measBody.z.Data);
title('Robot Motion')
% legend('Torque Ctrl')
xlabel('Time [s]');
ylabel('Z Position [m]');
subplot(3,1,3)
hold on
grid on
plot(simout_torque.measBody.vX.Time,simout_torque.measBody.vX.Data);
plot(simout_torque.measBody.vX.Time,arr_mean_vel);
title('Robot Motion')
% legend('Torque Ctrl');
xlabel('Time [s]');
ylabel('X Velocity [m/s]');

%% Joint Plots
jointNames = {'Rfront','Rrear','Lfront','Lrear'};
for idx = 1:4
    figure(idx+1)
    jName = jointNames{idx};
    
    % Joint angle
    subplot(3,2,1)
    hold on
    grid on
    plot(simout_torque.measLegs.(jName).AngleF.Time,simout_torque.measLegs.(jName).AngleF.Data);
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    title([jName ' angle femur'])

    subplot(3,2,2)
    hold on
    grid on
    plot(simout_torque.measLegs.(jName).AngleT.Time,simout_torque.measLegs.(jName).AngleT.Data);
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    title([jName ' angle tibia'])


    % Joint speed
    subplot(3,2,3)
    hold on
    grid on
    plot(simout_torque.measLegs.(jName).OmegaF.Time,simout_torque.measLegs.(jName).OmegaF.Data);
    xlabel('Time [s]');
    ylabel('Speed [rad/s]');
    title([jName ' speed femur'])

    subplot(3,2,4)
    hold on
    grid on
    plot(simout_torque.measLegs.(jName).OmegaT.Time,simout_torque.measLegs.(jName).OmegaT.Data);
    xlabel('Time [s]');
    ylabel('Speed [rad/s]');
    title([jName ' speed tibia'])

        
    % Joint torque
    subplot(3,2,5)
    hold on
    grid on
    plot(simout_torque.measLegs.(jName).TorqueF.Time,simout_torque.measLegs.(jName).TorqueF.Data);
    xlabel('Time [s]');
    ylabel('Torque [N*m]');
    ylim(max_torque*[-1, 1])
    title([jName ' torque femur'])

    subplot(3,2,6)
    hold on
    grid on
    plot(simout_torque.measLegs.(jName).TorqueT.Time,simout_torque.measLegs.(jName).TorqueT.Data);
    xlabel('Time [s]');
    ylabel('Torque [N*m]');
    ylim(max_torque*[-1, 1])
    title([jName ' torque tibia'])
end


%% Cleanup
% bdclose(mdlName)