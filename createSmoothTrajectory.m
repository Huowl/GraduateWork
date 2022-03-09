function [q,fem_der,tib_der] = createSmoothTrajectory(fem,tib,period,evalTimes)
% Generates cubic spline trajectory parameters given waypoints
% for 4 joints (femur_firts,tibia_first,femur_second,tibia_second) and total gait period
% 2022, Ivan Borisov, ITMO University

% Create necessary values for calculations
numPoints = numel(fem);
traj_times = linspace(0,period,numPoints);

% Calculate derivatives
dt = period/(numPoints-1);
fem_der = [0, 0.5*( diff(fem(1:end-1)) + diff(fem(2:end)) )/dt, 0];
tib_der = [0, 0.5*( diff(tib(1:end-1)) + diff(tib(2:end)) )/dt, 0];

% Set initial conditions
% Evaluate the trajectory at the start and halfway points for right and
% left legs, respectively
q = cubicpolytraj([fem;tib],traj_times,evalTimes,...
    'VelocityBoundaryCondition',[fem_der;tib_der]);
