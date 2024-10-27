classdef workingCollisionCheck < handle

    properties
        emergencyStopPressed = false;  % Class property
    end

    methods
        function self = workingCollisionCheck()
            clf	
            self.runRobot();
            
        end
    end

    methods 

        function runRobot(self)
            clf
            % 
            robot = TreeBot;
            robot.PlotAndColourRobot();
            % birdOnBranchPos = [1.8,0.5,0.4; 0,0,0];
            % birdOnBranchPoints = cloudPoints.loadPointClouds('birdOnBranch.ply', birdOnBranchPos(1,:));
            % initalTreeBotPos = [0.5,0.5, 0.2];
            % initalTreeBotDest = [1,0.3,0];

            birdOnBranchPos= [1,0.8,0.1;0,0,0];

            birdOnBranchPoints = cloudPoints.loadPointClouds('birdOnBranch.ply', birdOnBranchPos(1,:));

            birdHousePrintingOffset = [0,0.2,0];
                        
            birdhousePos1 = [1,0.5,0];
            birdhouseDest1 = [1.1,0.7,0.3];
                        
            birdhousePos2 = birdhousePos1 + birdHousePrintingOffset;
            birdhouseDest2 = birdhouseDest1 + birdHousePrintingOffset;
            
            currentPos = robot.model.fkine(robot.model.getpos).t.';
            trajM = [currentPos;birdhousePos1; birdhouseDest1; birdhousePos2; birdhouseDest2];
            self.detectES();

            %% Initalise trajectory
            for j = 2:height(trajM)

                for n = 1 
                    q1 = robot.model.ikcon(transl(trajM(j-1,:)));
                    q2 = robot.model.ikcon(transl(trajM(j,:)));
                    steps = 100;
                    qMatrix = jtraj(q1,q2,steps); % Obtaing the joint space trajectory
                    n = n+1;
                end
    
                
                
                n = 1;
    
                while n<=steps              
    
                    if ~self.CheckCollision(robot.model, birdOnBranchPoints)
                        robot.model.animate(qMatrix(n, :));
                        n=n+1;
                        pause(0.01)
    
                    end
    
                    if self.CheckCollision(robot.model, birdOnBranchPoints)
                                                
                       currentPos = robot.model.fkine(robot.model.getpos).t; %Updating matrix to continue from current position to desried destination
                        q1 = robot.model.ikcon(transl(currentPos));
                        q2 = robot.model.ikcon(transl((trajM(j-1,:))));
                        qMatrix = jtraj(q1,q2,steps);
                        n = 1;                   
                   end
    
                   pause(0.01);
    
                   if self.emergencyStopPressed
                       disp("Emergency stop button pressed!! Stopping Robot.");
                       return;
                   end
                   pause(0.1) 
    
                end
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
                disp("No Collision");
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