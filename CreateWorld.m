classdef CreateWorld < handle
  

    properties
        workspace = [-3 3 -3 3 0 3];
        robot;
        scale = 0.1;
        qinit = [];
    end


   methods
       function self = CreateWorld()
           self.BuildWorld();
           
       end
      %% Build the working environment
       function BuildWorld(self)
           

            linearDobotBase = [0,0.7,0;0,0,0];
            treeBotBase = [1,0.7,0;0,0,0];

            self.robot = LinearDobot;
            self.robot.model.base = transl(linearDobotBase(1,:));
            self.robot.PlotAndColourRobot;

            robot2 = TreeBot;
            robot2.model.base = transl(treeBotBase(1,:));
            robot2.PlotAndColourRobot();


            self.qinit = zeros(1,self.robot.model.n);
            
    
             hold on
               %applies a surface image to the floor
               axis(self.workspace);
               surf([-2.5,-2.5;2.5,2.5] ...
                   ,[-2.5,2.5;-2.5,2.5] ...
                   ,[0.01,0.01;0.01,0.01] ...
                   ,'CData',imread('ground.jpg') ...
                   ,'FaceColor','texturemap');
               hold on
               %applies a surface image to the back wall
    
               oneImg = imread('SideOne.jpg');
               oneRot = rot90(oneImg, -1);
               surf([2.5,2.5;-2.5,-2.5] ...
                   ,[2.5,2.5;2.5,2.5] ...
                   ,[0.01,2;0.01,2] ...
                   ,'CData', oneRot ...
                   ,'FaceColor','texturemap');
    
               hold on
               %rotates, then plots the image for the other wall
               twoImg = imread('SideTwo.jpg');
               twoRot = rot90(twoImg, -1);
               surf([-2.5,-2.5;-2.5,-2.5] ...
                   ,[-2.5,-2.5;2.5,2.5] ...
                   ,[0.01,2;0.01,2] ...
                   ,'CData',twoRot ...
                   ,'FaceColor','texturemap');
               hold on;
    
               PlaceObject('birdOnBranch.ply', [-1.2,0.9,1.5]);
    
               PlaceObject("cart.ply", [0,0,0]);
    
            self.robot.model.plot(self.qinit,'workspace', self.workspace,'scale',self.scale);
       end
   end
end
