classdef treeBotCollisionCheck < handle

    properties
        emergencyStopPressed = false;
        % % a = arduino;
        

    end

    methods 
        function self = treeBotCollisionCheck(trajM, birdOnBranchPos)
            	

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
            
        
            robot = TreeBot;
            robot.model.base = transl(-1.2, 0.5, 1);
            robot.PlotAndColourRobot();
            hold on

            % printingOffset = [0.06,0.05,0.06];

            birdhousePos1 = trajM(1,:); % - printingOffset;
            birdhousePos2 = trajM(3,:); % - printingOffset;

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
            
            % PlaceObject('tree.ply', [0.2,1.7,0]);
            
            currentPos = robot.model.fkine(robot.model.getpos).t.';
            % initialTreeBotPos = currentPos;

            partIndexCounter=1;

            self.detectES();

            qValues = [0, 0, 0, 0,0,0, 0;
                -1.38,0.06,0.04,-0.09,-0.43,-0.01,0; ...
                1.00, 0.1, 0.36,0.94,0.43,-0.01,0; ...
                -1.38,0.06,0.04,-0.09,0.43,-0.01,0; ...
                0.87, 0.56,0.31,1.22,0,0,0];




                % 1.50,0.12,0.53,1.08,0.43,-0.01,0];

        
            for j = 1:size(trajM, 1)

                initalTreeBotDest = trajM(2, :);
                
                % q1 = robot.model.ikcon(transl(initialTreeBotPos))
                % q2 = robot.model.ikcon(transl(initalTreeBotDest))
                q1 = qValues(j,:);
                q2 = qValues(j+1,:);
                steps = 100;
                qMatrix = jtraj(q1, q2, steps); % Obtaining the joint space trajectory

                % qMatrix = [-1.38,0.06,0.04,-0.09,-0.43,-0.01,0; ...
                %     1.00, 0.1, 0.36,0.94,0.43,-0.01,0; ...
                %     -1.38,0.06,0.04,-0.09,0.43,-0.01,0; ...
                %     1.50,0.12,0.53,1.08,0.43,-0.01,0];

                n = 1;
                while n <= steps

                    % self.readArduinoPin();
                    
                    if self.CheckCollision(robot.model, birdOnBranchPoints) % Detecting if there is a collision
                        
                        % currentPos = robot.model.fkine(robot.model.getpos).t; %Updating matrix to continue from current position to desried destination
                        % q1 = robot.model.ikcon(transl(currentPos));
                        % q2 = robot.model.ikcon(transl(initalTreeBotDest));                        
                        % qTempMatrix = jtraj(q1, q2, steps);
                        % robot.model.animate(qTempMatrix(n, :));
                        % % 
                        n = n + 1;
                        disp("inself.collision");
                        pause(0.01);

                    else
                        % q = [0, -pi/4, pi/6, 0,0,0, 0];
                        robot.model.animate(qMatrix(n, :));
                        n = n + 1;
                        pause(0.01);
                    end

                    if rem(partIndexCounter, 2) == 0
          
                        self.updatePartMovement(robot.model, qMatrix(n,:),vertiesMatrix{partIndexCounter/2},birdPartMatrix{partIndexCounter/2});
                        n = n + 1;
            
                    end  
        
                    if self.emergencyStopPressed %|| realESpressed
                        disp("Emergency stop button pressed!! Stopping Robot.");
                        return;
                    end
                    pause(0.1);
                end
                
                
                % initialTreeBotPos = initalTreeBotDest; % Updating position
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
                projectedPos = currentPos - 0.04; % clearway of 0.3

                q1 = robot.ikcon(transl(currentPos));
                q2 = robot.ikcon(transl(projectedPos));

                steps = 100;
                qMatrixCollision = jtraj(q1, q2, steps); %Creating a temporary trajectory of avoiding .ply file      
                    
                for o = 1:steps % Move the robot out of the way
                    robot.animate(qMatrixCollision(o, :));
                    pause(0.01);
                end
                pause(0.1);
    
             end


            if ~withinLimits
                crash = false;
            end
                    
        end

        
        function detectES(self)
            
            %% Software eStop
            f = figure('KeyPressFcn', @(src, event) self.detectedEM(src, event)); % Creating emergency stop figure
            PlaceObject('emergencyStopButton.ply');

        end
        
        function detectedEM(self,~, event)
            
            if strcmp(event.Key, 'space')  % Checking if the pressed key is the space bar
                self.emergencyStopPressed = true;

            end

            % realESpressed = readDigitalPin(arduino, 'D8');
            % 
            % if realESpressed == 1
            %     self.emergencyStopPressed = true;
            % end

            
            % if realESpressed == 1
            %     disp("Hardware Emergency Stop Pressed!")
            %     self.emergencyStopPressed = true;
            % end
        end

        function updatePartMovement(self, robot, qValues, verticies, birdMatrix)
            currentTransformationMatrix = robot.fkine(qValues);
            transformedVertices = [verticies,ones(size(verticies,1),1)]*currentTransformationMatrix.T';
            set(birdMatrix,'Vertices',transformedVertices(:,1:3));


        end

        function readArduinoPin(self)

            % realESpressed = readDigitalPin(arduino, 'D8')
            disp("In readArduino");
            % if realESpressed == 1
            %     self.emergencyStopPressed = true;
            % end
        end

    end
end