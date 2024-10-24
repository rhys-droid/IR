% main.m
clc;
clear;

% Create the world and environment
world = World();

% Get the robot from the world
robot = world.getRobot();

% Pass the robot to the printing function and start printing
printer = printing(robot);
