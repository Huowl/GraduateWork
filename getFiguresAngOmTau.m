% Copyright 2017-2019 The MathWorks, Inc.

%% Setup
% clc; 
% close all;
run_previous

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