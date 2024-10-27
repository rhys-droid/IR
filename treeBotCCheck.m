classdef treeBotCCheck < handle

    properties
        emergencyStopPressed = false;  % Class property
    end

    methods
        function self = treeBotCCheck()
            clf	
        end

        function runRobot(self, TreeBot, birdOnBranchPos, initalTreeBotPos, initalTreeBotDest)
            clf

            robot = TreeBot;
            % robot.PlotAndColourRobot();
            birdOnBranchPoints = cloudPoints.loadPointClouds('birdOnBranch.ply', birdOnBranchPos(1,:));

            %% Initialise trajectory
            q1 = robot.model.ikcon(transl(initalTreeBotPos));
            q2 = robot.model.ikcon(transl(initalTreeBotDest));
            steps = 100;
            qMatrix = jtraj(q1, q2, steps); % Obtaining the joint space trajectory

            self.detectES();
            n = 1;

            while n <= steps              
                if ~self.CheckCollision(robot, birdOnBranchPoints)
                    robot.model.animate(qMatrix(n, :));
                    n = n + 1;
                    pause(0.01);
                end

                if self.CheckCollision(robot, birdOnBranchPoints)
                    currentPos = robot.model.fkine(robot.model.getpos).t; % Updating matrix
                    q1 = robot.model.ikcon(transl(currentPos));
                    q2 = robot.model.ikcon(transl(initalTreeBotDest));
                    qMatrix = jtraj(q1, q2, steps);
                    n = 1;                   
                end

                pause(0.01);

                if self.emergencyStopPressed
                    disp("Emergency stop button pressed!! Stopping Robot.");
                    return;
                end
                pause(0.1);
            end
        end

        function crash = CheckCollision(~, robot, xyzLimits)
            currentPos = robot.model.fkine(robot.model.getpos).t; % Extracting current position as a 3D vector            
            withinXlim = (currentPos(1) >= xyzLimits(1, 1)) && (currentPos(1) <= xyzLimits(1, 2));
            withinYlim = (currentPos(2) >= xyzLimits(2, 1)) && (currentPos(2) <= xyzLimits(2, 2));
            withinZlim = (currentPos(3) >= xyzLimits(3, 1)) && (currentPos(3) <= xyzLimits(3, 2));
             
            withinLimits = withinXlim && withinYlim && withinZlim; 
        
            if withinLimits
                crash = true;
                disp("Crash");
                currentPos = robot.model.fkine(robot.model.getpos).t; 
                projectedPos = currentPos - 0.03; % clearway of 0.3
                q1 = robot.model.ikcon(transl(currentPos));
                q2 = robot.model.ikcon(transl(projectedPos));
                steps = 30;
                qMatrixCollision = jtraj(q1, q2, steps);      
                    
                for o = 1:steps % Move the robot out of the way
                    robot.model.animate(qMatrixCollision(o, :));
                    pause(0.01);
                end
            end

            if ~withinLimits
                disp("No Collision");
                crash = false;
            end
        end
        
        function detectES(self)
            f = figure('KeyPressFcn', @(src, event) self.detectedEM(src, event));
            PlaceObject('emergencyStopButton.ply');
        end
        
        function detectedEM(self,~, event)
            if strcmp(event.Key, 'space')  % Checking if the pressed key is the space bar
                self.emergencyStopPressed = true;
            end
        end
    end
end
