classdef treeBotCollisionCheck < handle

    properties
        emergencyStopPressed = false;  % Class property
    end

    methods
        function self = treeBotCollisionCheck()
            clf	
            self.runRobot();
            
        end
    end

    methods 

        function runRobot(self)
            clf

            birdOnBranchPoints = cloudPoints.loadPointClouds('birdOnBranch.ply', [1.8,0.5,0]);

            robot = TreeBot;
            robot.PlotAndColourRobot();
            
            %% Move Robot

             q1 = robot.model.ikcon(transl(0.5,0.5, 0.2));
             q2 = robot.model.ikcon(transl(1,0.3, 0));

            steps = 100;
            qMatrix = jtraj(q1,q2,steps); % Obtaing the joint space trajectory

            self.detectES();
            

            for n = 1:steps
               robot.model.animate(qMatrix(n, :));
               axis equal
               self.CheckCollision(robot.model, birdOnBranchPoints);

               if self.emergencyStopPressed
                   disp("Emergency stop button pressed!! Stopping Robot.");
                   return;
               end
               pause(0.1)            

            end
            
        end

        function CheckCollision(self,robot, xyzLimits)

            currentPos = robot.fkine(robot.getpos).t; % Extract the position as a 3D vector

            withinXlim = (currentPos(1) >= xyzLimits(1, 1)) && (currentPos(1) <= xyzLimits(1, 2));
            withinYlim = (currentPos(2) >= xyzLimits(2, 1)) && (currentPos(2) <= xyzLimits(2, 2));
            withinZlim = (currentPos(3) >= xyzLimits(3, 1)) && (currentPos(3) <= xyzLimits(3, 2));
             
            withinLimits = withinXlim && withinYlim && withinZlim; % Determine if the position is within all limits
        
            % Check for collision
            if withinLimits
                %disp("Crash");
            else
                %disp("No Collision");
            end
                    
        end

        
        function detectES(self)
            % Create a figure
            f = figure('KeyPressFcn', @(src, event) self.detectedEM(src, event));
            PlaceObject('emergencyStopButton.ply');
            
        end
        
        function detectedEM(self,~, event)
            % Check if the pressed key is the space bar
            if strcmp(event.Key, 'space')  
                % self.emergencyStopPressed = true;
                self.emergencyStopPressed = true;
            end
        end
    end
end


