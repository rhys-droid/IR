classdef World < handle
    properties
        workspace = [-3 3 -3 3 0 3];
        robot_dobot;
        % robot_treebot;
        scale = 0.1;
        qinit = [];
        qinit2 = [];
    end

   methods
       function self = World()
           clc;
           self.BuildWorld();
       end

      %% Build the working environment
       function BuildWorld(self)
            self.robot_dobot = LinearDobot;
            self.robot_dobot.model.base = trotx(pi/2) * transl(0, 1, 1.6);
            % self.qinit = zeros(1, self.robot_dobot.model.n);
            hold on;
            % self.robot_treebot = TreeBot;
            % self.robot_treebot.model.base = transl(-0.15, 0, 1);
            % self.qinit2 = zeros(1,self.robot_treebot.model.n);


            % self.robot.PlotAndColourRobot();
            % self.robot2.PlotAndColourRobot();
           



            hold on
            % applies a surface image to the floor
            axis(self.workspace);
            surf([-2.5,-2.5;2.5,2.5], [-2.5,2.5;-2.5,2.5], [0.01,0.01;0.01,0.01], 'CData',imread('ground.jpg'), 'FaceColor','texturemap');
            hold on

            % applies a surface image to the back wall
            oneImg = imread('SideOne.jpg');
            oneRot = rot90(oneImg, -1);
            surf([2.5,2.5;-2.5,-2.5], [2.5,2.5;2.5,2.5], [0.01,2;0.01,2], 'CData', oneRot, 'FaceColor','texturemap');

            % rotates, then plots the image for the other wall
            twoImg = imread('SideTwo.jpg');
            twoRot = rot90(twoImg, -1);
            surf([2.5,2.5;2.5,2.5], [2.5,2.5;-2.5,-2.5], [0.01,2;0.01,2], 'CData',twoRot, 'FaceColor','texturemap');
            hold on;

            % % bird on branch
            % [faceData, vertexData, ~] = plyread("birdOnBranch.ply");
            % Rz = [-1 0 0; 0 -1 0; 0 0 1];
            % vertexData = (Rz * vertexData')';
            % trisurf(faceData, vertexData(:, 1) - 2, vertexData(:, 2) + 1, vertexData(:, 3) + 1.5, 'FaceColor', 'none');

            % % trees
            PlaceObject('tallerTree.ply', [0.5,-1.2,0]);
            PlaceObject('tree.ply', [0.2,1.7,0]);
            % PlaceObject('tree.ply', [-2,2,0]);
            PlaceObject('birdOnBranch.ply', [0.2,-1.2,1.5]);

            % carts
            cartpos1 = [-1,-1.5,0];
            % cartpos2 = cartpos1 * [-1,-1,-1]';
            cartpos2 = [1.4,-1,0];
            cart1 = PlaceObject("cart.ply", cartpos1);
            cart2 = PlaceObject("cart.ply", cartpos2);
            vertsCart = [get(cart2,'Vertices'), ones(size(get(cart2,'Vertices'),1),1)] *trotz(pi);
            set(cart2,'Vertices',vertsCart(:,1:3));
            % [faceData, vertexData, ~] = plyread("cart.ply");
            % Rz = [-1 0 0; 0 -1 0; 0 0 1];
            % vertexData = (Rz * vertexData')';
            % trisurf(faceData, vertexData(:, 1)-0.37, vertexData(:, 2) + 0.48, vertexData(:, 3));

       end

       % Method to return the LinearDobot robot
       function robot = getRobot(self)
           robot = self.robot_dobot;
       end
       % function robot2 = getRobot2(self)
       %     robot2 = self.robot_treebot;
       % end

   end
end
