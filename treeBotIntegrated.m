classdef treeBotIntegrated < handle

    properties
        emergencyStopPressed = false;

    end

    methods 
        function self = treeBotIntegrated(trajM, birdOnBranchPos)
            clf	

            if nargin < 1 % Set default values if arguments are not provided
                error("Must initalise class object with desired variables")
            end
            if nargin < 2
                error("Must initalise class object with desired variables")  
            end
            self.runRobot(trajM, birdOnBranchPos);
            
        end
    end

    methods

        function runRobot(self, trajM, birdOnBranchPos)
            clf
        
            robot = TreeBot;
            
            robot.PlotAndColourRobot();
            hold on

            printingOffset = [0.06,0.05,0.06];

            birdhousePos1 = trajM(1,:) - printingOffset;
            birdhousePos2 = trajM(3,:) - printingOffset;

            % birdhousePos2 = birdhousePos1 + birdHousePrintingOffset;
            % birdhouseDest2 = birdhouseDest1 + birdHousePrintingOffset;
                        
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
            
            currentPos = robot.model.fkine(robot.model.getpos).t.';
            initialTreeBotPos = currentPos;

            partIndexCounter=1;

            self.detectES();
        
            for j = 1:size(trajM, 1)
                initalTreeBotDest = trajM(j, :);
                
                q1 = robot.model.ikcon(transl(initialTreeBotPos));
                q2 = robot.model.ikcon(transl(initalTreeBotDest));
                steps = 100;
                qMatrix = jtraj(q1, q2, steps); % Obtaining the joint space trajectory

                n = 1;
                while n <= steps

                    if self.CheckCollision(robot.model, birdOnBranchPoints) % Detecting if there is a collision
                        
                        currentPos = robot.model.fkine(robot.model.getpos).t; %Updating matrix to continue from current position to desried destination
                        q1 = robot.model.ikcon(transl(currentPos));
                        q2 = robot.model.ikcon(transl(initalTreeBotDest));
                        qMatrix = jtraj(q1, q2, steps);
                        n = 1;

                    else
                        robot.model.animate(qMatrix(n, :));
                        n = n + 1;
                        pause(0.01);
                    end

                    if rem(partIndexCounter, 2) == 0
          
                        self.updatePartMovement(robot.model, qMatrix(n,:),vertiesMatrix{partIndexCounter/2},birdPartMatrix{partIndexCounter/2});
                        n = n + 1;
            
                    end  
        
                    if self.emergencyStopPressed
                        disp("Emergency stop button pressed!! Stopping Robot.");
                        return;
                    end
                    pause(0.1);
                end
                
                
                initialTreeBotPos = initalTreeBotDest; % Updating position
                partIndexCounter=partIndexCounter+1;
            end
        end


        function crash = CheckCollision(~,robot, xyzLimits)

            currentPos = robot.fkine(robot.getpos).t; % Extracting current position as a 3D vector            

            withinXlim = (currentPos(1) >= xyzLimits(1, 1)) && (currentPos(1) <= xyzLimits(1, 2));
            withinYlim = (currentPos(2) >= xyzLimits(2, 1)) && (currentPos(2) <= xyzLimits(2, 2));
            withinZlim = (currentPos(3) >= xyzLimits(3, 1)) && (currentPos(3) <= xyzLimits(3, 2));
             
            withinLimits = withinXlim && withinYlim && withinZlim; %Finding the limits of the .ply file rather than looping through all the cloudPoints
        
            if withinLimits
                crash = true;
                disp("Crash");
                currentPos = robot.fkine(robot.getpos).t; 
                projectedPos = currentPos - 0.03; % clearway of 0.3

                q1 = robot.ikcon(transl(currentPos));
                q2 = robot.ikcon(transl(projectedPos));

                steps = 30;
                qMatrixCollision = jtraj(q1, q2, steps); %Creating a temporary trajectory of avoiding .ply file      
                    
                for o = 1:steps % Move the robot out of the way
                    robot.animate(qMatrixCollision(o, :));
                    pause(0.01);
                end
    
             end


            if ~withinLimits
                crash = false;
            end
                    
        end

        
        function detectES(self)

            f = figure('KeyPressFcn', @(src, event) self.detectedEM(src, event)); % Creating emergency stop figure
            PlaceObject('emergencyStopButton.ply');
            
        end
        
        function detectedEM(self,~, event)
            
            if strcmp(event.Key, 'space')  % Checking if the pressed key is the space bar
                self.emergencyStopPressed = true;

            end
        end

        function updatePartMovement(self, robot, qValues, verticies, birdMatrix)
            currentTransformationMatrix = robot.fkine(qValues);
            transformedVertices = [verticies,ones(size(verticies,1),1)]*currentTransformationMatrix.T';
            set(birdMatrix,'Vertices',transformedVertices(:,1:3));


        end
    end
end