function plottingDobot

     
    robot = LinearDobotMagician;
    robot.CreateModel();
    %robot.model.base = transl(linearUR3eBasePos(1,:))*trotx(linearUR3eBasePos(2,1))*troty(linearUR3eBasePos(2,2));
    
    % robot.PlotAndColourRobot;

    grid on
    axis equal

    axis([-4 4 -4 4 -4 4]);
    % scale = 0.5;

    % robot.plot('workspace', workspace, 'scale', scale);

    robot.PlotAndColourRobot;

    pause()

 

end