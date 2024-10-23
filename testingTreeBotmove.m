
robot = TreeBot;
robot.PlotAndColourRobot();

q1 = robot.model.ikcon(transl(0.5,-0.5, 0.2));
q2 = robot.model.ikcon(transl(1,0.3, 0));

steps = 100;
qMatrix = jtraj(q1,q2,steps); % Obtaing the joint space trajectory


for n = 1:steps
    robot.model.animate(qMatrix(n, :));
    axis equal
    pause(0.1) 
end



