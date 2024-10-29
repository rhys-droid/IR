birdOnBranchPosition = [0.5,-0.1,1.5;0,0,0];
           
birdhousePos1 = [-1.2,-0.4,0.9];
birdhouseDest1 = [0.05,1.2,1.3];

birdhousePos2 = [-1.2,-0.2,0.9];
birdhouseDest2 = [0.1,1,2];

% realESpressed = readDigitalPin(arduino, 'D8')

trajM = [birdhousePos1; birdhouseDest1; birdhousePos2; birdhouseDest2];

treeBotCollisionCheck(trajM, birdOnBranchPosition);
% hold on

% collisionChecker.runRobot()

axis equal

