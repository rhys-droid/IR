
birdOnBranchPosition = [1.1,1.3,0.1;0,0,0];

% birdHousePrintingOffset = [0,0.2,0];
% 
% birdhousePos1 = [1,0.5,0];
% birdhouseDest1 = [1.1,0.7,0.3];
% 
% birdhousePos2 = birdhousePos1 + birdHousePrintingOffset;
% birdhouseDest2 = birdhouseDest1 + birdHousePrintingOffset;
            
birdhousePos1 = [1,0.5,0];
birdhouseDest1 = [1.1,0.7,0.3];
            


%currentPos = robot.model.fkine(robot.model.getpos).t.';
trajM = [birdhousePos1; birdhouseDest1; birdhousePos2; birdhouseDest2];
collisionChecker = treeBotIntegrated(trajM, birdOnBranchPosition, birdhousePos1, birdhouseDest1);
hold on

collisionChecker.runRobot()

axis equal

