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

            birdOnBranchPoints = cloudPoints.loadPointClouds('birdOnBranch.ply', [1.8,0.3,0]);

            robot = TreeBot;
            robot.PlotAndColourRobot();
            
            %% Move Robot

             q1 = robot.model.ikcon(transl(0.8,0.5, -0.2));
             q2 = robot.model.ikcon(transl(0.6,1, -0.2));

            steps = 50;
            qMatrix = jtraj(q1,q2,steps); % obtaing the joint space trajectory

            for n = 1:steps
               robot.model.animate(qMatrix(n, :));
                axis equal
                treeBotCollisionCheck.CheckCollision(robot.model, birdOnBranchPoints)
               pause(0.01)            

            end
            
        end

        function CheckCollision(robot, currentCloudPoints)

            currentPos = robot.fkine(robot.getpos).t; % Extract the position as a 3D vector

            for i = 1:size(currentCloudPoints, 1) % Loop through the point cloud matrix
                % Calculate the Euclidean distance
                distance = norm(currentPos - currentCloudPoints(i, :)); % Calculate the distance
        
                if distance < 1  % Check if the distance is less than 1
                    disp("CRASH"); % Display crash message
                else
                    disp("Safe distance")
                    return;  % Exit the function after detecting a collision
                end
            end
        end
    end
end

