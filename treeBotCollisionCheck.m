classdef treeBotCollisionCheck < handle

%#ok<*NOPRT>

    methods
        function self = treeBotCollisionCheck()
			clf	
            self.Part1();
        end
    end

    methods (Static)

        function Part1()
            clf

            emergencyStopPressed = false;

            birdOnBranchPoints = cloudPoints.loadPointClouds('birdOnBranch.ply', [1.8,0.5,0]);

            robot = TreeBot;
            robot.PlotAndColourRobot();
            
            %% Move Robot

             q1 = robot.model.ikcon(transl(0.5,0.5, 0.2));
             q2 = robot.model.ikcon(transl(1,0.3, 0));

            steps = 100;
            qMatrix = jtraj(q1,q2,steps); % obtaing the joint space trajectory

            for n = 1:steps
               robot.model.animate(qMatrix(n, :));
               axis equal
               treeBotCollisionCheck.CheckCollision(robot.model, birdOnBranchPoints)
               if emergecnyButtonPressed
                   disp("stop");
               end
               pause(0.1)            

            end
            
        end

        function CheckCollision(robot, xyzLimits)

            currentPos = robot.fkine(robot.getpos).t; % Extract the position as a 3D vector

            withinXlim = (currentPos(1) >= xyzLimits(1, 1)) && (currentPos(1) <= xyzLimits(1, 2));
            withinYlim = (currentPos(2) >= xyzLimits(2, 1)) && (currentPos(2) <= xyzLimits(2, 2));
            withinZlim = (currentPos(3) >= xyzLimits(3, 1)) && (currentPos(3) <= xyzLimits(3, 2));
             
            withinLimits = withinXlim && withinYlim && withinZlim; % Determine if the position is within all limits
        
            % Check for collision
            if withinLimits
                disp("Crash");
            else
                disp("No Collision");
            end
                    
        end

        function emergencyStopPressed
            if spacebar
                emergencyStopPressed = true;
            else
                emergencyStopPressed = false;
            end
        end
    end
end

