clc;
clear;

dobot_home_position = [-1.05, 0.95, 1.3]; 
treebot_home_position = [-1.2, 0.5, 1.0]; 

global startPrintingFlag;
startPrintingFlag = false;

function triggerPrinting()
    global startPrintingFlag;
    startPrintingFlag = true;
end

gui_control = GUIintegration(@triggerPrinting);

world = World();
robot_dobot = world.getRobot();
robot_treebot = world.getRobot2();
printer = printing(robot_dobot);



robot_dobot.model.plot(world.qinit, 'workspace', world.workspace, 'scale', world.scale);
robot_treebot.model.plot(world.qinit2, 'workspace', world.workspace, 'scale', world.scale);

target_positions = [0, 1.1, 1.3; -1.5, 1.5, 0];

birdOnBranchPosition = [0.45,-0.55,1.5;0,0,0];
birdhousePos1 = [-1.2,-0.4,0.9];
birdhouseDest1 = [0.05,1.2,1.3];
birdhousePos2 = [-1.2,-0.2,0.9];
birdhouseDest2 = [0.1,1,2];
trajM = [birdhousePos1; birdhouseDest1; birdhousePos2; birdhouseDest2];
axis equal;

while true
    if ~gui_control.EstopPressed
        pause(0.1);

        if startPrintingFlag
            startPrintingFlag = false;  
            disp('Starting Birdhouse Printing Sequence...');
            
            num_birdhouses = size(printer.birdhouse_positions, 1);
            for i = 1:num_birdhouses
                fprintf('Starting print and pickup sequence for birdhouse %d...\n', i);
                printer.printBirdhouse(i);
                
                [pickup_position, birdhouse_index] = printer.getBirdhousePosition(i);
                fprintf('Printing complete. Fetching pickup position for birdhouse %d at (%.2f, %.2f, %.2f)\n', ...
                    birdhouse_index, pickup_position(1), pickup_position(2), pickup_position(3));
                                
                pickup = treeBotCollisionCheck(trajM, birdOnBranchPosition, robot_treebot);
                pickup.runRobot(trajM, birdOnBranchPosition);
                
                disp(['Birdhouse ', num2str(i), ' successfully picked up and placed at target.']);
                if gui_control.EstopPressed
                    disp('Emergency Stop activated. Exiting sequence.');
                    break; 
                end

                pause(1);
            end

            disp('All birdhouses printed and placed successfully.');
        end

        if gui_control.EstopPressed
            disp('Emergency Stop activated. Exiting...');
            break;
        end
    else
        pause;
    end
end
