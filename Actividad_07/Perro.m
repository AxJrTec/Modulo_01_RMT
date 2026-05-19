%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.02;               % Sample time [s]
tVec = 0:sampleTime:138;         % Time array

initPose = [4;-4;pi];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [4,-4; 4,-5; 1,-5; 0,-3; 0,-1; 0,0; -1,1; -4,1; -3,2; -2,2; ...
            -3,2; -4,1; -5,3; -5,4; -4,4; -5,3; -4,4; -3,4; -3,5; -2,5; ...
            -1,5; -2,4; -2,5; -3,5; -1,6; -1,7; 0,6; 1,6; 1,5; 1,7; 2,4; 
             1,3; 2,4; 1,7; 3,3; 4,2; 0,0; 0,-1; 5,1; 4,2];

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 4.3;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
 
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    %waitfor(r);   
end