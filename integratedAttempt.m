birdOnBranchPosition = [0.2,-1.2,1.5;0,0,0];
           
birdhousePos1 = [-1.2,-1.1,0.9];
birdhouseDest1 = [0,1.1,1.3];

birdhousePos2 = [-1.2,-0.9,0.9];
birdhouseDest2 = [-2,1.7,0];

% realESpressed = readDigitalPin(arduino, 'D8')

trajM = [birdhousePos1; birdhouseDest1; birdhousePos2; birdhouseDest2];

treeBotCollisionCheck(trajM, birdOnBranchPosition);
% hold on

% collisionChecker.runRobot()

axis equal

