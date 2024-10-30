classdef treeBotCollisionCheck < handle

    properties
        emergencyStopPressed = false;
        treebot; 
    end

    methods 
        function self = treeBotCollisionCheck(trajM, birdOnBranchPos, treebotInstance)
         
            
            if nargin < 2 
                error("Must initialize class object with desired variables")
            end
            
            
            self.treebot = treebotInstance;
            % self.runRobot(trajM, birdOnBranchPos);
        end
    end

    methods
        function runRobot(self, trajM, birdOnBranchPos)
            hold on;

            birdhousePos1 = trajM(1,:);
            birdhousePos2 = trajM(3,:); 

            birdhousePart1 = PlaceObject("birdhouse.ply", birdhousePos1);
            hold on
            birdhousePart2 = PlaceObject("birdhouse.ply", birdhousePos2);
            hold on
                        
            vertsBhouse1 = get(birdhousePart1,'Vertices');
            set(birdhousePart1, 'Vertices',vertsBhouse1(:,1:3));
            hold on
                        
            vertsBhouse2 = get(birdhousePart2,'Vertices');
            set(birdhousePart2, 'Vertices',vertsBhouse2(:,1:3));
            hold on
                        
            vertsBhouse1 = vertsBhouse1 - birdhousePos1;
            vertsBhouse2 = vertsBhouse2- birdhousePos2;
            
            vertiesMatrix = {vertsBhouse1;vertsBhouse2};
            birdPartMatrix = {birdhousePart1;birdhousePart2};

            birdOnBranchPoints = cloudPoints.loadPointClouds('birdOnBranch.ply', birdOnBranchPos(1,:));
 


            % 
            % birdhousePos1 = trajM(1, :);
            % birdhousePos2 = trajM(2, :);
            % 
            % 
            % birdhousePart1 = PlaceObject("birdhouse.ply", birdhousePos1);
            % birdhousePart2 = PlaceObject("birdhouse.ply", birdhousePos2);
            % hold on;
            % 
            % vertsBhouse1 = get(birdhousePart1, 'Vertices');
            % vertsBhouse2 = get(birdhousePart2, 'Vertices');
            % % vertiesMatrix = {vertsBhouse1 - birdhousePos1; vertsBhouse2 - birdhousePos2};
            %     %     Rz = [cosd(90), -sind(90), 0; sind(90), cosd(90), 0; 0, 0, 1];
            %     % 
            %     % 
            %     birdhousePart1 = PlaceObject("birdhouse.ply", birdhousePos1);
            %     birdhousePart2 = PlaceObject("birdhouse.ply", birdhousePos2);
            %     % 
            %     % 
            %     % vertsBhouse1 = (Rz * get(birdhousePart1, 'Vertices')')';
            %     % vertsBhouse2 = (Rz * get(birdhousePart2, 'Vertices')')';
            %     % 
            %     % 
            %     vertsBhouse1 = vertsBhouse1 - birdhousePos1;
            %     vertsBhouse2 = vertsBhouse2 - birdhousePos2;
            % 
            % 
            %     set(birdhousePart1, 'Vertices', vertsBhouse1);
            %     set(birdhousePart2, 'Vertices', vertsBhouse2);
            %     % 
            %     % vertiesMatrix = {vertsBhouse1; vertsBhouse2};
            %         birdPartMatrix = {birdhousePart1; birdhousePart2};
            % 
            % 
            % birdOnBranchPoints = cloudPoints.loadPointClouds('birdOnBranch.ply', birdOnBranchPos(1, :));
            % 
            currentPos = self.treebot.model.fkine(self.treebot.model.getpos).t.';
            initialTreeBotPos = currentPos;

           qValues = [0, 0, 0, 0,0,0, 0;
                -1.38,0.06,0.04,-0.09,-0.43,-0.01,0; ...
                1.00, 0.1, 0.36,0.94,0.43,-0.01,0; ...
                -1.38,0.06,0.04,-0.09,0.43,-0.01,0; ...
                0.87, 0.56,0.31,1.22,0,0,0];

            partIndexCounter = 1;
       
            for j = 1:size(trajM, 1)
                initalTreeBotDest = trajM(j, :);

             
                % q1 = self.treebot.model.ikcon(transl(initialTreeBotPos));
                % q2 = self.treebot.model.ikcon(transl(initalTreeBotDest));
                % steps = 100;
                % qMatrix = jtraj(q1, q2, steps); 
               q1 = qValues(j,:);
               q2 = qValues(j+1,:);
               steps = 100;
               qMatrix = jtraj(q1, q2, steps); % Obtaining the joint space trajectory

                n = 1;
                while n <= steps
                    if self.CheckCollision(birdOnBranchPoints) 
                       
                        currentPos = self.treebot.model.fkine(self.treebot.model.getpos).t;
                        q1 = self.treebot.model.ikcon(transl(currentPos));
                        q2 = self.treebot.model.ikcon(transl(initalTreeBotDest));
                        qMatrix = jtraj(q1, q2, steps);
                        n = 1;
                    else
                        self.treebot.model.animate(qMatrix(n, :));
                        n = n + 1;
                        pause(0.01);
                    end

   

                    if rem(partIndexCounter, 2) == 0

                        self.updatePartMovement(qMatrix(n,:),vertiesMatrix{partIndexCounter/2},birdPartMatrix{partIndexCounter/2});
                        n = n + 1;

                    end  

                    if self.emergencyStopPressed 
                        disp("Emergency stop button pressed!! Stopping Robot.");
                        return;
                    end
                    pause(0.1);
                end


                % initialTreeBotPos = initalTreeBotDest; 
                partIndexCounter=partIndexCounter+1;


            end
        end

        function crash = CheckCollision(self, xyzLimits)
      
            currentPos = self.treebot.model.fkine(self.treebot.model.getpos).t;
            
         
            withinXlim = (currentPos(1) >= xyzLimits(1, 1)) && (currentPos(1) <= xyzLimits(1, 2));
            withinYlim = (currentPos(2) >= xyzLimits(2, 1)) && (currentPos(2) <= xyzLimits(2, 2));
            withinZlim = (currentPos(3) >= xyzLimits(3, 1)) && (currentPos(3) <= xyzLimits(3, 2));
            crash = withinXlim && withinYlim && withinZlim;

            if crash
                disp("Collision detected, moving robot to avoid obstacle.");
                projectedPos = currentPos - 0.03;
                qMatrixCollision = jtraj(self.treebot.model.getpos(), self.treebot.model.ikcon(transl(projectedPos)), 30);
                self.treebot.model.animate(qMatrixCollision);
            end
        end


        
        function detectedEM(self, ~, event)
            if strcmp(event.Key, 'space')
                self.emergencyStopPressed = true;
            end
        end

        function updatePartMovement(self, qValues, vertices, birdMatrix)
            currentTransformationMatrix = self.treebot.model.fkine(qValues);
            transformedVertices = [vertices, ones(size(vertices, 1), 1)] * currentTransformationMatrix.T';
            set(birdMatrix, 'Vertices', transformedVertices(:, 1:3));
        end


    end
end
