robotParameters;
kv = 0.0131;
M = rotor_inertia + femur_mass * femur_length^2 / 2;

% A = zeros(2,2);
% 
% 
% A(1,1) = -rotor_damping / M;
% A(1,2) = torque_c / M;
% A(2,1) = - kv / arm_inductance;
% A(2,2) = - arm_resistance;
% 
% B = zeros(2,1);
% B = [0; (m_voltage/max_torque)];
% 
% Q = zeros(2,2);
% Q(1,1) = 100;
% Q(2,2) = 10;
% 
% R = 5;
% 
% [K,S,e] = lqr(A,B,Q,R);

% A = zeros(3,3);
% 
% A(1,2) = 1;
% A(2,2) = -rotor_damping / M;
% A(2,3) = torque_c / M;
% A(3,2) = - kv / arm_inductance;
% A(3,3) = - arm_resistance;
% 
% B = zeros(3,1);
% B = [0; 0; (m_voltage/max_torque)];
% 
% Q = zeros(3,3);
% Q(1,1) = 10^4;
% Q(2,2) = 10^6;
% Q(3,3) = 1e5;
% 
% R = 20;
% 
% [K,S,e] = lqr(A,B,Q,R);

Tu = 1 / PWM_freq;
K = rotor_damping + torque_c * kv / arm_resistance;
Kd = rotor_inertia / 2 / Tu;
Kp = K / 2 / Tu;

Kd = Kd - Kp*Tu;