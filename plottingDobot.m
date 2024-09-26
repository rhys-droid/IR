function plottingDobot

     
    robot = LinearDobotMagician;
    robot.CreateModel();
    %robot.model.base = transl(linearUR3eBasePos(1,:))*trotx(linearUR3eBasePos(2,1))*troty(linearUR3eBasePos(2,2));
    
    robot.PlotAndColourRobot;

    axis equal

    %look at when gavin sets up the work space to then plot the robot

    pause()

 

end