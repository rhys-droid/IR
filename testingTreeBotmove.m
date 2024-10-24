
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
        disp("Moving")

        if rem(m, 2) ~= 0
            
            currentTransformationMatrix = robot.model.fkine(qMatrix(n,:));
            currentEndEff = robot.model.fkine(robot.model.getpos).t;

           %if needing to pick up different names parts, use counting system "birdhouse" +num2string(n) + '.ply'
            pause(0.01);
           
            transformedVertices = [vertsBhouse1,ones(size(vertsBhouse,1),1)]*currentTransformationMatrix.T';
            %transformedVertices = currentTransformationMatrix.T';
            set(birdhousePart1,'Vertices',transformedVertices(:,1:3));
            pause(0.01)    
        end

    end

     
     pause(0.01);
     %disp("Moved")

end



