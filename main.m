% main.m
clc;
clear;

% Create the world and environment
world = World();

% Get the robot from the world
robot_dobot = world.getRobot();
robot_treebot = world.getRobot2();
% 
% robot_dobot.PlotAndColourRobot();
% robot_treebot.PlotandColourRobot();

robot_dobot.model.plot(world.qinit, 'workspace', world.workspace, 'scale', world.scale);
robot_treebot.model.plot(world.qinit2, 'workspace', world.workspace, 'scale', world.scale);

% Pass the robot to the printing function and start printing
printer = printing(robot_dobot);

% After printing is done, pick up the prints
pickup = PickUp(robot_treebot);
pickup.startPickUp();