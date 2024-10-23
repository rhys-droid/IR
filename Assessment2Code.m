classdef Assessment2Code < handle
  

    properties
        workspace = [-2 2 -2 2 0 2];
        robot;
        scale = 0.1;
        qinit = [];
    end


   methods
       function self = Assessment2Code()
           self.BuildWorld();
           
       end
      %% Build the working environment
       function BuildWorld(self)

        self.robot = LinearDobotMagician;
        self.robot.PlotAndColourRobot;
        self.qinit = zeros(1,self.robot.model.n);
        

         hold on
           %applies a surface image to the floor
           axis(self.workspace);
           surf([-1.8,-1.8;1.8,1.8] ...
               ,[-1.8,1.8;-1.8,1.8] ...
               ,[0.01,0.01;0.01,0.01] ...
               ,'CData',imread('ground.jpg') ...
               ,'FaceColor','texturemap');
           hold on
           %applies a surface image to the back wall

           oneImg = imread('SideOne.jpg');
           oneRot = rot90(oneImg, -1);
           surf([1.5,1.5;-1.5,-1.5] ...
               ,[1.5,1.5;1.5,1.5] ...
               ,[0.01,2;0.01,2] ...
               ,'CData', oneRot ...
               ,'FaceColor','texturemap');

           hold on
           %rotates, then plots the image for the other wall
           twoImg = imread('SideTwo.jpg');
           twoRot = rot90(twoImg, -1);
           surf([-1.5,-1.5;-1.5,-1.5] ...
               ,[-1.5,-1.5;1.5,1.5] ...
               ,[0.01,2;0.01,2] ...
               ,'CData',twoRot ...
               ,'FaceColor','texturemap');
           hold on;

           PlaceObject('birdOnBranch.ply', [-1.2,0.9,1.5]);

        self.robot.model.plot(self.qinit,'workspace', self.workspace,'scale',self.scale);
       end
   end
end
