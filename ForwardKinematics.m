function [x,z] = ForwardKinematics(HipAngle,KneeAngle)

t = 0.2; % [m]
f = 0.2; % [m]

a = HipAngle;
b = KneeAngle;

psi_20 = zeros(3,numel(a));
for idx = 1:numel(a)
    T01 = [cos(a(idx)), -sin(a(idx)) 0; ...
            sin(a(idx)),  cos(a(idx)) 0; ...
            0 0  1];
    T12 = [cos(b(idx)), -sin(b(idx)) f; ...
            sin(b(idx)),  cos(b(idx)) 0; ...
            0 0 1];
    
    psi_20(:,idx) = T01*T12*[t 0 1]';
end
x = psi_20(1,:);
z = psi_20(2,:);
end