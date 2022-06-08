function [c , ceq] = nonlinearContraints(x)

x = 3.*x;
N = (numel(x)-5)/4;
gait_period = 1;
fem_motionFront = deg2rad([x(1:N), x(1)]);
tib_motionFront = deg2rad([x(N+1:2*N), x(N+1)]);
fem_motionRear = deg2rad([x(2*N+1:3*N), x(2*N+1)]);
tib_motionRear = deg2rad([x(3*N+1:4*N), x(3*N+1)]);

numTrajPoints = 101;
evalTimes = linspace(0,gait_period,numTrajPoints);
[q_rr,~,~] = createSmoothTrajectory(fem_motionRear,tib_motionRear,gait_period,evalTimes);
[q_fr,~,~] = createSmoothTrajectory(fem_motionFront,tib_motionFront,gait_period,evalTimes);
[x_fr, z_fr] = ForwardKinematics(q_fr(1,:),q_fr(2,:));
[x_r, z_r] = ForwardKinematics(q_rr(1,:),q_rr(2,:));

S = [x_r; z_r]';
count1 = countIntersection(S);
S = [x_fr; z_fr]';
count2 = countIntersection(S);
c = count1 + count2 - 2;
ceq = [];
end