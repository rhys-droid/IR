% main.m
clc;
clear;

world = World();

% Get the robot from the world
robot_dobot = world.getRobot();
robot_treebot = world.getRobot2();

% 
% robot_dobot.PlotAndColourRobot();
% robot_treebot.PlotandColourRobot();

robot_dobot.model.plot(world.qinit, 'workspace', world.workspace, 'scale', world.scale);
robot_treebot.model.plot(world.qinit2, 'workspace', world.workspace, 'scale', world.scale);

num_birdhouses = size(printer.birdhouse_positions, 1);

spmd
    if (spmdSend == 1)
printer = printing(robot_dobot);   

for i = 1:num_birdhouses
   printer.printBirdhouse(i);
   spmdSend(i, 2);
   pause(1);
end

    elseif (spmdSend == 2)
    pickup = PickUp(robot_treebot); 
    for i = 1:num_birdhouses
        birdhouse_num = spmdReceive(1);
        if (i == 1)
        target_position = [-0.2, 2, 1.2];

        else
             target_position = [0.2, 2, 1.2];
        end
    
        start_pos = [printer.birdhouse_positions(birdhouse_num, :)]; 
        picker.pickUpBirdhouse(start_pos, target_position);
        pause(1);

    end
    end
end
