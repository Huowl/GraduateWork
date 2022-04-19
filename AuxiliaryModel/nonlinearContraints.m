function [c , ceq] = nonlinearContraints(x)

l1 = 0.15;
l2 = 0.15;

Thk = @(q)[cos(q) -sin(q) l1; ...
           sin(q) cos(q) 0; ...
           0 0 1];
Tke = @(q)[cos(q) -sin(q) l2; ...
           sin(q) cos(q) 0; ...
           0 0 1];

N = numel(x)/4; 
fem_motionFront = deg2rad(x(1:N));
tib_motionFront = deg2rad(x(N+1:2*N));
fem_motionRear = deg2rad(x(2*N+1:3*N));
tib_motionRear = deg2rad(x(3*N+1:4*N));

psi_heRear = zeros(3,N);
psi_heFront = zeros(3,N);
for idx = 1:N
    psi_heRear(:,idx) = Thk(fem_motionRear(idx))*Tke(tib_motionRear(idx))*[0 0 1]';
    psi_heFront(:,idx) = Thk(fem_motionFront(idx))*Tke(tib_motionFront(idx))*[0 0 1]';
end
c = [0.2 + psi_heFront(1,:) - 0.35, - 0.1 + psi_heFront(2,:) + 0.35...
    - 0.2 + psi_heRear(1,:) + 0.35, - 0.1 + psi_heFront(2,:) + 0.35];
ceq = [];
end