clf
clc
%% Define plyFiles and robot base

% birdOnBranchPos = [3,2,0;0,0,0];
linearDobotBase = [0,0.2,0; pi/2, 0,0]; % xyz corrdinates; xyz rotation;
threeDOfBase = [1,0,0];
% birdOnBranch_h = PlaceObject('birdOnBranch.ply', birdOnBranchPos(1,:));
% hold on

%% Plot robot and cloud poitns for branch to avoid

% ptCloud = pcread('birdOnBranch.ply'); %Reading PLY point clouds
% 
% birdOnBranchPtCloudTrans = pointCloud(ptCloud.Location+birdOnBranchPos(1,:)); % Translating point clouds to desired position
% 
% pcshow(birdOnBranchPtCloudTrans); % Showing translated point clouds
% 
% birdOnBranchXYZLimits = [birdOnBranchPtCloudTrans.XLimits; birdOnBranchPtCloudTrans.YLimits; birdOnBranchPtCloudTrans.ZLimits];
% 
% hold on

%% Create 6DOF robot

L1 = Link('d',0.3,'a',0,'alpha',-pi/2,'qlim',[-2*pi 2*pi]);
L2 = Link('d', 0, 'a', 0.4, 'alpha', 0, 'qlim', [-pi/2, 0]);
L3 = Link('d', 0, 'a', 0.4, 'alpha', 0, 'qlim', [0, pi/2]);
L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi, pi]);
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/4, 5*pi/4]);
L6 = Link('d', 0.2, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi, pi]);
L7 = Link('d', 0.2, 'a', 0, 'alpha', pi/2, 'qlim', [-pi, pi]);
L8 = Link('d', 0.2, 'a', 0, 'alpha', 0, 'qlim', [-2*pi, 2*pi]);

robot = SerialLink([L1, L2, L3, L4, L5, L6, L7, L8], 'name','myRobot');

q = [0, -pi/4, pi/6, 0,0,0, 0, 0];

scale = 0.2;
workspace = [-0.5 1.5 -0.5 1.5 0 1];
robot.base = transl(threeDOfBase);
robot.plot(q, 'workspace', workspace, 'scale', scale); 

axis equal


pause(0.1);

hold on

%% Plotting Linear Dobot for scale
% 
% robot = LinearDobot;
% robot.model.base = transl(linearDobotBase(1,:)) * trotx(linearDobotBase(2,1)) * troty(linearDobotBase(2,2))* trotz(linearDobotBase(2,3));
% robot.PlotAndColourRobot;

%robot = TreeBot;

hold off


axis equal