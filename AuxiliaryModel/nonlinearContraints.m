function [c , ceq] = nonlinearContraints(x)

l1 = 0.2;
l2 = 0.15;

N = (numel(x)-2)/4; 
fem_motionFront = deg2rad(x(1:N));
tib_motionFront = deg2rad(x(N+1:2*N));
fem_motionRear = deg2rad(x(2*N+1:3*N));
tib_motionRear = deg2rad(x(3*N+1:4*N));

numTrajPoints = 101;
evalTimes = linspace(0,gait_period,numTrajPoints);
[q_fr,~,~] = createSmoothTrajectory(femur_motionFront,tibia_motionFront,gait_period,evalTimes);
[q_rr,~,~] = createSmoothTrajectory(femur_motionRear,tibia_motionRear,gait_period,evalTimes);

[x_fr z_fr] = ForwardKinematics(q_fr(1,:),q_fr(2,:));
[x_r z_r] = ForwardKinematics(q_rr(1,:),q_rr(2,:));

c = [0.2 + psi_heFront(1,:) - 0.35, - 0.1 + psi_heFront(2,:) + 0.35...
    - 0.2 + psi_heRear(1,:) + 0.35, - 0.1 + psi_heFront(2,:) + 0.35];
ceq = [];
end