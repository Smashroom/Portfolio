% Walking Robot Parameters
% Copyright 2017 The MathWorks, Inc.

%% General parameters
density = 1000;
foot_density = 2000;
world_damping = 0.25;
world_rot_damping = 0.25;
if ~exist('actuatorType','var')
    actuatorType = 1;
end

%% Inputs
gaitPeriod = 0.8;
time = linspace(0,gaitPeriod,7)';
ankle_motion = deg2rad([-7.5 10 10 5 0 -10 -7.5]');
%ankle_motion = deg2rad([0.218166156499291;0.218166156499291;0.174532925199433;-0.436332312998582;0;-0.0436332312998582;0.218166156499291]); 
%ankle_motion = deg2rad([[-0.436332312998582;-0.436332312998582;-0.130899693899575;0.305432619099008;0.305432619099008;-0.392699081698724;-0.436332312998582]]);

knee_motion = deg2rad([10, -5, 2.5, -10, -10, 15, 10]');
%knee_motion =deg2rad([0.218166156499291;1.35263017029561;0.610865238198015;0;0.0872664625997165;0.523598775598299;0.218166156499291]);
%knee_motion = deg2rad([0.785398163397448;0;0.349065850398866;0.392699081698724;0.567232006898157;0.130899693899575;0.785398163397448]]);
hip_motion = deg2rad([-10, -7.5, -15, 10, 15, 10, -10]');
%hip_motion =deg2rad([0.741764932097590;0.654498469497874;-0.654498469497874;0.261799387799149;0.261799387799149;0.698131700797732;0.741764932097590]);%
%hip_motion = deg2rad([[-0.567232006898157;0.610865238198015;0.654498469497874;-0.610865238198015;0.0872664625997165;0.305432619099008;-0.567232006898157]]);
curveData = createSmoothTrajectory(ankle_motion,knee_motion,hip_motion,gaitPeriod);

%% Contact/friction parameters
contact_stiffness = 2500;
contact_damping = 100;
mu_k = 0.6;
mu_s = 0.8;
mu_vth = 0.1;
height_plane = 0.025;
plane_x = 25;
plane_y = 3;
contact_point_radius = 1e-4;

%% Foot parameters
foot_x = 8;
foot_y = 6;
foot_z = 1;
foot_offset = [-1 0 0];

%% Leg parameters
leg_radius = 0.75;
lower_leg_length = 10;
upper_leg_length = 10;

%% Torso parameters
torso_y = 10;
torso_x = 5;
torso_z = 8;
torso_offset_z = -2;
torso_offset_x = -0.5;
init_height = foot_z + lower_leg_length + upper_leg_length + ...
              torso_z/2 + torso_offset_z + height_plane/2;

%% Joint parameters
joint_damping = 1;
joint_stiffness = 1;
motion_time_constant = 0.01; %0.025;

%% Joint controller parameters
hip_servo_kp = 60;
hip_servo_ki = 10;
hip_servo_kd = 20;
knee_servo_kp = 60;
knee_servo_ki = 5;
knee_servo_kd = 10;
ankle_servo_kp = 20;
ankle_servo_ki = 4;
ankle_servo_kd = 8;
deriv_filter_coeff = 100;
max_torque = 20;

%% Electric motor parameters
motor_resistance = 1;
motor_constant = 0.02;
motor_inertia = 0;
motor_damping = 0;
motor_inductance = 1.2e-6;
gear_ratio = 50;