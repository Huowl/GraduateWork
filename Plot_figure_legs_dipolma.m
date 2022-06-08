% % clear all; clc; close all;
% 
% addpath(genpath('ResultOptimize18-04'))
% addpath(genpath('ResultOptimRigid27_04'))
% 
% path = 'walk_5_PD_ctrl_optimizedData_20Apr22_1233';
% load(path)
% 
traj_times = linspace(0,gait_period,7);
% 
%Evaluate the trajectory at a few points for visualization
numTrajPoints = 101;
evalTimes = linspace(0,gait_period,numTrajPoints);
[q_fr,fem_der_front,tib_der_front] = createSmoothTrajectory(femur_motionFront,tibia_motionFront,gait_period,evalTimes);
[q_rr,fem_der_rear,tib_der_rear] = createSmoothTrajectory(femur_motionRear,tibia_motionRear,gait_period,evalTimes);

%Plot the resulting trajectory
h1 = figure;
h1.Position(3:4) = [1920 1080];
set(h1,'DefaultAxesFontSize',14,'DefaultAxesFontName','Times New Roman');
subplot(2,1,1)
plot(evalTimes,rad2deg(q_fr(1,:)),'b-',traj_times,rad2deg(femur_motionFront),'ro');
xlabel('Время [с]');
ylabel('Угол бед. сустава [град]');
grid on;
subplot(2,1,2)
plot(evalTimes,rad2deg(q_fr(2,:)),'b-',traj_times,rad2deg(tibia_motionFront),'ro');
xlabel('Время [с]');
ylabel('Угол колена [град]');
grid on;
saveas(gcf, '..\picture\h1','png');

h2 = figure;
h2.Position(3:4) = [1920 1080];
set(h2,'DefaultAxesFontSize',14,'DefaultAxesFontName','Times New Roman');
subplot(2,1,1)
plot(evalTimes,rad2deg(q_rr(1,:)),'b-',traj_times,rad2deg(femur_motionRear),'ro');
xlabel('Время [с]');
ylabel('Угол бед. сустава [град]');
grid on;
subplot(2,1,2)
plot(evalTimes,rad2deg(q_rr(2,:)),'b-',traj_times,rad2deg(tibia_motionRear),'ro');
xlabel('Время [с]');
ylabel('Угол колена [град]');
grid on;
saveas(gcf, '..\picture\h2','png');

[x z] = ForwardKinematics(q_fr(1,:),q_fr(2,:));
h3 = figure;
h3.Position(3:4) = [720 720];
set(h3,'DefaultAxesFontSize',14,'DefaultAxesFontName','Times New Roman');
plot(x,z,'b-');
xlabel('Координата по оси Х, [м]');
ylabel('Координата по оси Z, [м]');
grid on;
saveas(gcf, '..\picture\h3f','png');

[x_r z_r] = ForwardKinematics(q_rr(1,:),q_rr(2,:));
% func = zeros(1,numel(x_r));
% for idx = 1:numel(x_r)
%         func(x_r(func == 1) == x_r(idx) && z_r(func == 1) == z_r(idx)) = 2;
%         func(idx) = 1;
% end

h35 = figure;
h35.Position(3:4) = [720 720];
set(h3,'DefaultAxesFontSize',14,'DefaultAxesFontName','Times New Roman');
% plot(x_r,z_r,'b-',x_r(func==2),z_r(func==2),'r*');
plot(x_r,z_r,'b-')
xlabel('Координата по оси Х, [м]');
ylabel('Координата по оси Z, [м]');
grid on;
saveas(gcf, '..\picture\h3r','png');


GrContact = prev_simout.ContactGround;
RR = GrContact.LegRR;
RF = GrContact.LegRF;
LR = GrContact.LegLR;
LF = GrContact.LegLF;

h4 = figure;
h4.Position(3:4) = [640 360];
set(h4,'DefaultAxesFontSize',12,'DefaultAxesFontName','Times New Roman');
hold on;
plot(RR.Time(RR.Data ~= 0),RR.Data(RR.Data ~= 0),...
    '--w|', 'MarkerSize',10,'MarkerEdgeColor',[0.1 0.1 0.8],'MarkerFaceColor',[0.1 0.1 0.8])
hold on;
plot(RF.Time(RF.Data ~= 0),RF.Data(RF.Data ~= 0)*1.5,...
    '--w|', 'MarkerSize',10,'MarkerEdgeColor',[0.8 0.1 0.1],'MarkerFaceColor',[0.8 0.1 0.1])
plot(LR.Time(LR.Data ~= 0),LR.Data(LR.Data ~= 0)*2,...
    '--w|', 'MarkerSize',10,'MarkerEdgeColor',[1 0.8 0],'MarkerFaceColor',[1 0.8 0])
plot(LF.Time(LF.Data ~= 0),LF.Data(LF.Data ~= 0)*2.5,...
    '--w|', 'MarkerSize',10,'MarkerEdgeColor',[0.2 0.8 0],'MarkerFaceColor',[0.2 0.8 0])
xlabel('Время, [с]');
ax = gca;
ax.YTick = [1, 1.5, 2, 2.5];
ax.YTickLabel = {'ПрЗ', 'ПрП', 'ЛЗ', 'ЛП'};
ax.YLim = [0.9, 2.6];
saveas(gcf, '..\picture\h4','png');



PlotCoM_xyz;
saveas(gcf, '..\picture\h5','png');
close all;