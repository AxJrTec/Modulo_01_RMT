%% EXAMPLE: Differential Drive Path Following
% In this example, a differential drive robot navigates a set of waypoints 
% using the Pure Pursuit algorithm while avoiding obstacles using the
% Vector Field Histogram (VFH) algorithm.
% 
% Copyright 2019 The MathWorks, Inc.

%% Simulation setup
% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);

% Sample time and time array
sampleTime = 0.05;              % Sample time [s]
tVec = 0:sampleTime:100;        % Time array

% Initial conditions
initPose = [7;2;0];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;


% Load map

%complexMap       41x52                2132  logical              
%emptyMap         26x27                 702  logical              
%simpleMap        26x27                 702  logical              
%ternaryMap      501x501            2008008  double  

close all
load complexmap

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi, pi, 360); % -pi a pi es un círculo completo (360 rayos)
lidar.maxRange = 1;%5

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Path planning and following

% Create waypoints
initPose = [7;2;0];
waypoints = [

% ===== P1: INICIO =====
7 2;

% ===== SUBIDA BLOQUE IZQUIERDO =====
7 4;
7 6;
7 8;

5 8;
3 8;

1.5 10;
1.5 12;
1.5 14;
5 14;
8 14;
8 16;

% ===== P2 =====
3 18;

% ===== RECORRIDO SUPERIOR =====
5 18;
7 18;
9 14;
11 14;
13 14;

15 14;
16 15;
17 19.1;
17.5 19.1;
18 19.1;
18.5 19.1;
19 19.1;
19.5 19.1;
20 19.1;
20.5 19.1;
21 19.1;
21.5 19.1;
22 18;

% ===== P3 =====
22 17;

% ===== BAJADA BLOQUE DERECHO =====
24.5 15;
24 13;
24 11;
24 9;
19.5 8.5;
16.5 8.5;
16 6;
16.5 4.5
18.5 4.5
21.5 4.5



% ===== P4 =====
24 1
];
% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;%0.5
controller.DesiredLinearVelocity = 1.5; %0.75
controller.MaxAngularVelocity = 3;

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.01 1]; %0.05 3
vfh.NumAngularSectors = 100; %36
vfh.HistogramThresholds = [5 10]; % 5y 10
vfh.RobotRadius = L;
vfh.SafetyDistance = L;
vfh.MinTurningRadius = 0.1;%0.25

%% Simulation loop
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    % Actualizar visualización solo cada 20 iteraciones para acelerar la animación
    %if mod(idx, 20) == 0
        viz(pose(:,idx), waypoints, ranges)
    %end
    waitfor(r);
end