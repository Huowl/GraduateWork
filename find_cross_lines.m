% S = [ -1 2 3 2 -1 3 0 1 1;...
%      1 4 -3 2 2 6 -3 5 8]';
% sortS = zeros(size(S));
% 
% sortS(:,2) = sort(S(:,2));

numTrajPoints = 101;
evalTimes = linspace(0,gait_period,numTrajPoints);
[q_rr,fem_der_rear,tib_der_rear] = createSmoothTrajectory(femur_motionRear,tibia_motionRear,gait_period,evalTimes);
[q_fr,fem_der_front,tib_der_front] = createSmoothTrajectory(femur_motionFront,tibia_motionFront,gait_period,evalTimes);
[x_r z_r] = ForwardKinematics(q_rr(1,:),q_rr(2,:));
[x_fr z_fr] = ForwardKinematics(q_fr(1,:),q_fr(2,:));

S = [x_fr; z_fr]';
c = 0;
for idx1 = 1:size(S,1)-1
    for idx2 = (idx1+2):size(S,1)-1
        denominator = (S(idx2+1,2)-S(idx2,2))*(S(idx1,1)-S(idx1+1,1)) - (S(idx2+1,1)-S(idx2,1))*(S(idx1,2)-S(idx1+1,2));
        num_a = (S(idx2+1,1)-S(idx1+1,1))*(S(idx2+1,2)-S(idx2,2)) - (S(idx2+1,1)-S(idx2,1))*(S(idx2+1,2)-S(idx1+1,2));
        num_b = (S(idx1,1)-S(idx1+1,1))*(S(idx2+1,2)-S(idx1+1,2)) - (S(idx2+1,1)-S(idx1+1,1))*(S(idx1,2)-S(idx1+1,2));
        Ua = num_a / denominator;
        Ub = num_b / denominator;
        if (Ua >= 0 && Ua <= 1 && Ub >= 0 && Ub <=1)
            c = c+1;
        end
    end
end