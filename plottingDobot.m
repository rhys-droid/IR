function plottingDobot

     
    robot = DobotMagician;
    robot.CreateModel();
    %robot.model.base = transl(linearUR3eBasePos(1,:))*trotx(linearUR3eBasePos(2,1))*troty(linearUR3eBasePos(2,2));
    robot.PlotAndColourRobot;
    pause()
    %editing the code
        

end