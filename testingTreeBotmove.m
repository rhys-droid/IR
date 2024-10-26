classdef testingTreeBotmove < handle

    properties
        partIndex = 1;
        partIndexIncrease = false;
    end

    methods
        function self = testingTreeBotmove()
            clc
            self.runRobot();
            
        end
    end

    methods 

        function runRobot(self)


            robot = TreeBot;
            robot.PlotAndColourRobot();
            hold on
            
            birdHousePrintingOffset = [0,0.2,0];
            
            birdhousePos1 = [1,0.5,0];
            birdhouseDest1 = [1.1,0.7,0.3];
            
            birdhousePos2 = birdhousePos1 + birdHousePrintingOffset;
            birdhouseDest2 = birdhouseDest1 + birdHousePrintingOffset;
            
            birdhousePart1 = PlaceObject("birdhouse.ply", birdhousePos1);
            birdhousePart2 = PlaceObject("birdhouse.ply", birdhousePos2);
            
            vertsBhouse1 = get(birdhousePart1,'Vertices');
            set(birdhousePart1, 'Vertices',vertsBhouse1(:,1:3));
            
            vertsBhouse2 = get(birdhousePart2,'Vertices');
            set(birdhousePart2, 'Vertices',vertsBhouse2(:,1:3));
            
            vertsBhouse1 = vertsBhouse1 - birdhousePos1;
            vertsBhouse2 = vertsBhouse2- birdhousePos2;
            
            vertiesMatrix = {vertsBhouse1;vertsBhouse2};
            birdPartMatrix = {birdhousePart1;birdhousePart2};

            
            hold on
            
            currentPos = robot.model.fkine(robot.model.getpos).t.';
            trajM = [currentPos; birdhousePos1; birdhouseDest1; birdhousePos2; birdhouseDest2];
            axis equal
            
            
            for m = 2:height(trajM)
                            
                q1 = robot.model.ikcon(transl(trajM(m-1,:)));
                q2 = robot.model.ikcon(transl(trajM(m,:)));
                steps = 50;
                qMatrix = jtraj(q1,q2,steps);
            
                for n = 1:steps
            
                    robot.model.animate(qMatrix(n, :));
                    axis equal
                    pause(0.01)
            

                    if rem(m, 2) ~= 0

                        i = self.partIndex;
                        self.updatePartMovement(robot.model, qMatrix(n,:),vertiesMatrix{i},birdPartMatrix{i});
            
                    end   
            
                 end
            
                if self.partIndexIncrease
                
                    self.partIndex = self.partIndex +1;
                    self.partIndexIncrease = false;
                
                end
            
            
            end
        end

        function updatePartMovement(self, robot, qValues, verticies, birdMatrix)
            currentTransformationMatrix = robot.fkine(qValues);
            transformedVertices = [verticies,ones(size(verticies,1),1)]*currentTransformationMatrix.T';
            set(birdMatrix,'Vertices',transformedVertices(:,1:3));
            self.partIndexIncrease = true;

        end
    end
end



