
birdOnBranchPosition = [1,1,0.1;0,0,0];
           
birdhousePos1 = [1,0.5,0];
birdhouseDest1 = [1.1,0.7,0.3];

trajM = [birdhousePos1; birdhouseDest1; birdhousePos2; birdhouseDest2];
collisionChecker = treeBotIntegrated(trajM, birdOnBranchPosition, birdhousePos1, birdhouseDest1);
hold on

collisionChecker.runRobot()

axis equal

