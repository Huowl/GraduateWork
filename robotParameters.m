%% robotParameters
% First version
% ground = [20 2 0.1]; % [m]
% z_offset = 0.4; % [m]
% body_dim = [0.46 0.22 0.06]; % [m]
% body_mass = 2.0; % [kg]

% Equal Unitree A1 Feb 22 
ground = [20 2 0.1]; % [m]
z_offset = 0.4; % [m]
% body_dim = [0.267 0.194 0.114]; % [m]
% body_mass = 6.0; % [kg]
% femur_length = 0.15; % [m]
% tibia_length = 0.15; % [m]

%% Model Robot Unitree A1 March 6 2022

body_dim = [0.267 0.194 0.114]; % [m]
body_mass = 6.0; % [kg]

femur_length = 0.2; % [m]
femur_width = 0.0245;
femur_height = 0.034;
femur_mass = 0.888; %[kg]

femParam = [femur_length femur_width femur_height femur_mass];

tibia_length = 0.2; % [m]
tibia_width = 0.016;
tibia_height = 0.016;
tibia_mass = 0.151; %[kg]

tibParam = [tibia_length tibia_width tibia_height tibia_mass];

hip_radius = 0.046;
hip_length = 0.04;
hip_mass = 0.595;
hip_offset = 0.065;

leg_offset_x = 0.1805;
leg_offset_y = 0.047;

foot_mass = 0.06;

offset_RF = [leg_offset_x, -leg_offset_y, body_dim(3)/2];
offset_LF = [leg_offset_x, leg_offset_y, body_dim(3)/2];

offset_RR = [-leg_offset_x, -leg_offset_y, body_dim(3)/2];
offset_LR = [-leg_offset_x, leg_offset_y, body_dim(3)/2];

%% Motor Parameters Unitree A1 20 march 2022

PWM_freq = 4000; % [Hz]
max_torque = 33.5; % [V] Scaling PWM 100
m_voltage = 36; % [V] Output PWM

arm_resistance = 4.1730; %[Ohm]
arm_inductance = 250e-6; % [H]
torque_c = 0.8372; %[N*m/A]

rotor_inertia = 72e-6;
rotor_damping = 0.09;

g_ratio = 1.5;%10.3;


%% Spine parameters

bd_spine_eq_pos = 0; %[rad] equlibrium position spine
bd_spine_stiffness = 300; % [Hm/rad] spine joing spring stifness
bd_spine_damping = 10; %  [Hm/(rad/s)] spine joint damphing coefficient
% Bounds [rad]
spine_pos_low_limit = -pi/2;
spine_pos_up_limit = pi/2;
%% Contact
sp_rad = 0.02;
pl_x = ground(1);
pl_y = ground(2);
pdepth = 0.03;

cntstiff = 5e3; %1e5 past 03.04
cntdmp = 1e2; % 1e5

staticFriction = 1;
dynamicFriction = 0.9;
%%
world_damping = 20;     % Translational damping
world_rot_damping = 10; % Rotational damping
tsTraj = 0.01;