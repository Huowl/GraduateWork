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
femur_mass = 0.888;

tibia_length = 0.2; % [m]
tibia_width = 0.016;
tibia_height = 0.016;
tibia_mass = 0.151;

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
%% Contact
sp_rad = 0.02;
pl_x = ground(1);
pl_y = ground(2);
pdepth = 0.03;

cntstiff = 1e5;
cntdmp = 1e5;

staticFriction = 1;
dynamicFriction = 0.9;
%%
world_damping = 20;     % Translational damping
world_rot_damping = 10; % Rotational damping
tsTraj = 0.01;      

%% Control legs

k_p_fem = 80;
k_d_fem = 1;

k_i_fem = 350;

k_p_tib = 80;
k_d_tib = 1;

k_i_tib = 275;

max_torque = 100;