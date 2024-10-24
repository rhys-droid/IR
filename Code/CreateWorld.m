classdef CreateWorld < handle
  

    properties
        workspace = [-3 3 -3 3 0 3];
        robot;
        robot2;
        scale = 0.1;
        qinit = [];
        qinit2 = [];
    end


   methods
       function self = CreateWorld()
           clc;
           self.BuildWorld();
           
       end
      %% Build the working environment
       function BuildWorld(self)
           

            self.robot = LinearDobot;
            self.robot.model.base = trotx(pi/2) * transl(0, 0.92, 1.6);
            self.robot.PlotAndColourRobot;
            hold on;
            self.robot2 = TreeBot;
            self.robot2.model.base = transl(0.15, 0, 0.9);
            self.robot2.PlotAndColourRobot();


            self.qinit = zeros(1,self.robot.model.n);
            self.qinit2 = zeros(1,self.robot2.model.n);
    
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
    
           
    
            [faceData, vertexData, ~] = plyread("birdOnBranch.ply");
            Rz = [-1 0 0; 0 -1 0; 0 0 1];
            vertexData = (Rz * vertexData')';
            trisurf(faceData, vertexData(:, 1) - 2, vertexData(:, 2) + 1, vertexData(:, 3) + 1.5, 'FaceColor', 'none');

               %carts
               PlaceObject("cart.ply", [0,-2,0]);
            [faceData, vertexData, ~] = plyread("cart.ply");
            Rz = [-1 0 0; 0 -1 0; 0 0 1];
            vertexData = (Rz * vertexData')';
            trisurf(faceData, vertexData(:, 1)-0.37, vertexData(:, 2) + 0.48, vertexData(:, 3));
               
    
            self.robot.model.plot(self.qinit,'workspace', self.workspace,'scale',self.scale);
            self.robot2.model.plot(self.qinit2,'workspace', self.workspace,'scale',self.scale);
       end
   end
end
