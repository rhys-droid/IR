classdef treeBotCollisionCheck < handle

    properties
        emergencyStopPressed = false;
    end

    methods 
        function self = treeBotCollisionCheck(trajM, birdOnBranchPos)
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

            birdOnBranchPoints = cloudPoints.loadPointClouds('birdOnBranch.ply', birdOnBranchPos(1,:));
            
            currentPos = robot.model.fkine(robot.model.getpos).t.';
            initialTreeBotPos = currentPos;

            self.detectES();
        
            for j = 1:size(trajM, 1)
                initalTreeBotDest = trajM(j, :);
                
                q1 = robot.model.ikcon(transl(initialTreeBotPos));
                q2 = robot.model.ikcon(transl(initalTreeBotDest));
                steps = 100;
                qMatrix = jtraj(q1, q2, steps); % Obtaining the joint space trajectory
        
                
        
                n = 1;
                while n <= steps
                    % if ~self.CheckCollision(robot.model, birdOnBranchPoints)
                    %     robot.model.animate(qMatrix(n, :));
                    %     n = n + 1;
                    %     pause(0.01);
                    % end

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
        
                    if self.emergencyStopPressed
                        disp("Emergency stop button pressed!! Stopping Robot.");
                        return;
                    end
                    pause(0.1);
                end
                
                
                initialTreeBotPos = initalTreeBotDest; % Updating position
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
    end
end