% main.m
clc;
clear;

world = World();

world.BuildWorld();



birdOnBranchPosition = [0.2,-1.2,1.5;0,0,0];
           
birdhousePos1 = [-1.2,-0.4,0.9];
birdhouseDest1 = [0,1.1,1.3];

birdhousePos2 = [-1.2,-0.2,0.9];
birdhouseDest2 = [-1.5,1.5,0];

% birdHousePrintingOffset = [0.1,0.5,0];
% 
% birdhousePos2 = birdhousePos1 + birdHousePrintingOffset;
% birdhouseDest2 = birdhouseDest1 + birdHousePrintingOffset;



% realESpressed = readDigitalPin(arduino, 'D8')

trajM = [birdhousePos1; birdhouseDest1; birdhousePos2; birdhouseDest2];
hold on
treeBotCollisionCheck(trajM, birdOnBranchPosition);
hold on

% collisionChecker.runRobot()

axis equal

% Get the robot from the world
% robot_dobot = world.getRobot();
% birdOnBranchPosition = [1,1.5,0.1;0,0,0];
%robot_treebot = world.getRobot2();

% 
% robot_dobot.PlotAndColourRobot();
% robot_treebot.PlotandColourRobot();

% robot_dobot.model.plot(world.qinit, 'workspace', world.workspace, 'scale', world.scale);
%robot_treebot.model.plot(world.qinit2, 'workspace', world.workspace, 'scale', world.scale);


% spmd
%     if (spmdSend == 1)
% printer = printing(robot_dobot);   
% 
% for i = 1:num_birdhouses
%    printer.printBirdhouse(i);
%    spmdSend(i, 2);
%    pause(1);
% end

    % elseif (spmdSend == 2)
    % % pickup = PickUp(robot_treebot); 
    % for i = 1:num_birdhouses
    %     birdhouse_num = spmdReceive(1);
    %     if (i == 1)
    %     target_position = [-0.2, 2, 1.2];
    % 
    %     else
    %          target_position = [0.2, 2, 1.2];
    %     end
    % 
    %     start_pos = [printer.birdhouse_positions(birdhouse_num, :)]; 
    %     %picker.pickUpBirdhouse(start_pos, target_position);
    %     trajM = [start_pos;target_position];
    %     hold on 
    %     collisionChecker = treeBotCollisionCheck(trajM, birdOnBranchPosition);
    %     hold on        
    %     collisionChecker.runRobot()
    % 
    %     pause(1);
    % 
    % end
    % end
% end
