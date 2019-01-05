%  This file is an example template you can use for your solution.  It
% provides wrapper functions as well as various utilities that may be
% useful for your implementation
%
%  Your solution can implement the following function
%
%   [x,y] = bug_planner( qstart, qgoal )
%
% This function is expected to return a collision free path
% starting from qstart and ending in qgoal, both supplied as row
% vectors. The returned values should be row vectors with the x and
% y coordinates of the robot along this path, as it would have
% followed with your planner in effect.
%
% In implementing these functions, you are only allowed to use the
% following functions supplied with the homework (as well as any
% others you may choose to implement yourself):
%
%  draw_arena, draw_range_map, read_sensor
%
%  Furthermore, you are only allowed to access the following global
% variables:
%
%  sensor_range, infinity
%
%  You may of course write your own functions as you see fit as long
% as you do not cheat by accessing the global arena map and follow
% submission guidelines for this homework. You may also use various
% builtin Matlab functions as you see fit.
%
%  Please follow good coding standards and document your code with
% useful and clear comments in English.
%
% All quantities are in MKS units unless otherwise specified.
%  distances    : m
%  angles       : rad
%  speed        : m/s
%  acceleration : m/s^2
%

function ceng786_hw1;

%tic;

% Global Parameters Declarations -----------------
global sensor_range;  % Determines limited sensor range
global arena_limits;  % Boundaries of the arena: [xmin xmax ymin ymax]
global arena_map;     % Description of obstacles in the environment
global infinity;      % Large value to be used as 'infinity'
global qstart qgoal;  % Start and goal configurations
global planner;

% Parameter values to be used for the homework ---
qgoal = [6 3]; %[9 9];
qstart = [2 9]; %[0 0]; 
sensor_range = 0.5;
infinity = 1e5;
arena_limits = [0 10 0 10];
arena_map = [];
                                                                    
%User defined parameters
planner.show = 0; % To visualize the robot motion planning
planner.state = 0;
planner.qcurr = qstart;
%The robot closest distance to the object boundary
planner.safeDist = 0.1;
planner.minStep = 1e-4;
planner.obstacleMap = [];
planner.flag = 0;
planner.rayAngle = pi/180;
planner.lpt = 0;
planner.lptIndex = 0;
planner.ind = 0;
planner.finished = 0;
planner.errorMap = [];
planner.bugType = 1; % 1--> Bug1, 2--> Bug2 Algorithm
planner.qcurr = qstart;
planner.errorFlag = 0;
planner.mMap = []; % Mline map
planner.mLine = mod(atan2(qgoal(2)-qstart(2),qgoal(1)-qstart(1))+2*pi,2*pi);
% Invoking your solutions for the example arena ------------------------
init_arena();
xData = [];yData = [];
tic
if planner.bugType == 1
    while 1
        if planner.finished == 1
            break;
        end
        [x_m1_b1, y_m1_b1] = bug1_planner( qstart, qgoal );
        if planner.errorFlag == 1
            disp("Error Occurred");
            disp("Robot can not go to goal, please change the starting position or obstacles");
            while 1
                continue;
            end
        end
        xData = [xData x_m1_b1];
        yData = [yData y_m1_b1];
        planner.qcurr = [x_m1_b1(end), y_m1_b1(end)];
    end
else
    tic
    while 1
        if planner.finished == 1
            break;
        end
        if planner.errorFlag == 1
            disp("Error Occurred");
            disp("Robot can not go to goal, please change the starting position or obstacles");
            while 1
                continue;
            end
        end
        [x_m1_b1, y_m1_b1] = bug2_planner( qstart, qgoal );
        xData = [xData x_m1_b1];
        yData = [yData y_m1_b1];
        planner.qcurr = [x_m1_b1(end), y_m1_b1(end)];
    end
end
disp("Execution time of the algorithm is:")
toc
figure(1);
clf;
draw_arena;
hold on; plot( xData, yData);
if planner.show == 1
    figure(2);
    clf;
    for i = 1:length(xData)
        draw_range_map( [xData(i) yData(i)], 30 );
        drawnow;
    end
end

end
% -----------------------------------------------------------------
% init_arena
%
% Definition of the example arena map for Homework 1
%
% -----------------------------------------------------------------
function init_arena;
global arena_map qstart qgoal;

arena_map = [];
arena_map{1} = ...
[ 2.0392  3.5234; 1.8318  5.7751; 2.0161  6.7982; 2.6152  8.1433; ...
  3.5369  8.9035; 5.0576  9.0205; 6.3249  8.8158; 7.4078  7.8509; ...
  8.0300  6.8275; 8.1452  4.8977; 8.0760  3.4357; 7.6613  2.1784; ...
  6.8548  1.1842; 5.3571  0.8041; 4.3433  1.0965; 3.6751  2.5000; ...
  3.5369  3.7281; 3.4447  4.8099; 3.9977  6.2135; 4.5968  6.7982; ...
  5.0115  6.5936; 4.5968  5.6287; 4.1820  4.6930; 4.0668  3.7865; ...
  4.1129  2.5877; 4.5276  1.7398; 5.4954  1.3596; 6.6705  1.6228; ...
  7.2465  2.4415; 7.6843  3.6696; 7.7765  5.1608; 7.6843  6.3304; ...
  6.9700  7.3246; 6.1866  8.1140; 5.0346  8.3480; 3.7673  8.2018; ...
  3.1682  7.7924; 2.4539  6.7105; 2.3848  5.1023];
arena_map{2} = ...
[ 5.2889  5.1131; 4.7111  4.2839; 4.8869  3.5302; ...
  6.1683  3.9070; 6.1432  5.0377 ];
arena_map{3} = ...
[ 5.1382    7.2487; 5.3392  6.8719; 5.3392  6.3693; ...
    5.2889  6.0427; 5.5402  6.0930; 5.7412  6.4447; ...
    5.7412  6.9724; 5.4899  7.2990; 5.2638  7.4749 ];

% Bug2 outperforms Bug1
% arena_map{1} = [...
%     3.6996    9.1392;
%     2.8816    8.2967;
%     2.8205    7.0513;
%     3.3944    6.2821;
%     4.6398    7.0024;
%     4.6276    8.2601; ];
% arena_map{2} = [...
%     6.2149    6.4652;
%     5.2137    6.1600;
%     4.9939    5.5861;
%     4.7619    5.0000;
%     5.7509    4.5360;
%     6.4103    5.1221;];



% Bug1 outperforms Bug2
% arena_map{1} = [ 8.0009    2.6113
%     8.0102    1.9712
%     2.0176    1.9898
%     1.9991    8.0102
%     8.0102    8.0195
%     8.0009    3.9879
%     4.0121    3.9787
%     3.9842    4.6095
%     7.4907    4.5631
%     7.5093    7.4443
%     2.5000    7.4536
%     2.5093    2.6391
%     7.2124    2.7226];

% Soundness
% arena_map{1} = [3    7.001
%     7.001    7
%     7    3.001
%     3.001    3];

end