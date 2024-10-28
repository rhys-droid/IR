
birdOnBranchPosition = [1,1.5,0.1;0,0,0];
           
birdhousePos1 = [1,0.5,0];
birdhouseDest1 = [1.1,0.7,0.3];

% birdHousePrintingOffset = [0.1,0.5,0];
% 
% birdhousePos2 = birdhousePos1 + birdHousePrintingOffset;
% birdhouseDest2 = birdhouseDest1 + birdHousePrintingOffset;

birdhousePos2 = [-0.1, 0.7, 0.1];
birdhouseDest2 = [1, 0.6, 0];

trajM = [birdhousePos1; birdhouseDest1; birdhousePos2; birdhouseDest2];
collisionChecker = treeBotCollisionCheck(trajM, birdOnBranchPosition);
hold on

collisionChecker.runRobot()

axis equal

