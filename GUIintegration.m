classdef GUIintegration < handle
    properties
        treebot; 
        app;    
        q;      
        stepSize = 0.01;
        direction = 'x';
        currentPos = [];
        newPos = [];
        EstopPressed = false;
        startPrintingCallback;
    end

    methods
        % Constructor
        function self = GUIintegration(startPrintingCallback)
            self.treebot = TreeBot;
            clf; 

            self.q = zeros(1, 7);  
            self.startPrintingCallback = startPrintingCallback; 
            %self.runRobot();  
            self.openGUI();  
        end

        % Initialize the TreeBot robot
        % function runRobot(self)
        %     addpath('TreeBot');  % Add the robot folder to path
        % 
        %     % Create the robot object
        %     self.robot = TreeBot();
        % 
        %     % Plot the robot with the initial configuration
        %     self.robot.PlotAndColourRobot();
        %     self.robot.AnimateRobot(self.q);  % Plot the robot at initial pose
        % 
        %     % Ensure the sliders reflect the robot's initial pose
        %     self.updateSliders();
        % end

        function openGUI(self)
            self.app = GUI();  

           
            self.app.Link1Slider.ValueChangingFcn = @(src, event) self.updateJoint(1, event.Value);
            self.app.Link2Slider.ValueChangingFcn = @(src, event) self.updateJoint(2, event.Value);
            self.app.Link3Slider.ValueChangingFcn = @(src, event) self.updateJoint(3, event.Value);
            self.app.Link4Slider.ValueChangingFcn = @(src, event) self.updateJoint(4, event.Value);
            self.app.Link5Slider.ValueChangingFcn = @(src, event) self.updateJoint(5, event.Value);
            self.app.Link6Slider.ValueChangingFcn = @(src, event) self.updateJoint(6, event.Value);

            self.app.XButton.ButtonPushedFcn = @(~, ~) self.moveEndEffector('x');
            self.app.XButton_2.ButtonPushedFcn = @(~, ~) self.moveEndEffector('-x');
            self.app.YButton.ButtonPushedFcn = @(~, ~) self.moveEndEffector('y');
            self.app.YButton_2.ButtonPushedFcn = @(~, ~) self.moveEndEffector('-y');
            self.app.ZButton.ButtonPushedFcn = @(~, ~) self.moveEndEffector('z');
            self.app.ZButton_2.ButtonPushedFcn = @(~, ~) self.moveEndEffector('-z');

            self.app.JogamountmEditField.ValueChangedFcn = @(src, event) self.updateJog(event.Value);

            self.app.Button.ButtonPushedFcn = @(~, ~) self.updateEstop(true);

            self.app.ResumeButton.ButtonPushedFcn = @(~, ~) self.updateEstop(false);
            
            % self.app.BirdhouseButton.ButtonPushedFcn = @(src, event) self.runProgram('Birdhouse');
            self.app.BirdhouseButton.ButtonPushedFcn = @(~, ~) self.startPrinting();


            
        end

            function startPrinting(self)
                disp('Print Birdhouse button pressed.');
                self.startPrintingCallback(); 
            end



        function runProgram(self, product)
            if strcmp(product, 'Birdhouse')
                disp('YIPPEEEE');
            end
            if strcmp(product, 'Beehive')
                disp('RAAAHHHHH');
            end
        end



        function updateJog(self, inputVal)
            self.stepSize = inputVal;
        end

        function updateEstop(self, pressState)
            self.EstopPressed = pressState;  
            if self.EstopPressed
                disp('E-stop activated!');
            else
                disp('Resuming from E-stop.');
            end
        end



      
        function updateJoint(self, jointIndex, value)
         
            self.q(jointIndex) = deg2rad(value);
            try
                self.treebot.model.animate(self.q);
            catch
                warning('Animation failed.');
            end
        end

       
        function updateSliders(self)
        
            qDegrees = rad2deg(self.q);

           
            self.app.Link1Slider.Value = qDegrees(1);
            self.app.Link2Slider.Value = qDegrees(2);
            self.app.Link3Slider.Value = qDegrees(3);
            self.app.Link4Slider.Value = qDegrees(4);
            self.app.Link5Slider.Value = qDegrees(5);
            self.app.Link6Slider.Value = qDegrees(6);
        end




        function moveEndEffector(self, pressedDirect)
            
            self.direction = pressedDirect;

            self.currentPos = self.treebot.model.fkine(self.q);
            currentJoints = self.treebot.model.ikcon(self.currentPos.T);
                
            disp(self.stepSize);
            disp(self.direction);

            switch self.direction
                case 'x'

                       
                    self.newPos = self.currentPos.T;

                    
                    self.newPos(1, 4) = self.newPos(1, 4) + self.stepSize;
                    newJoints = self.treebot.model.ikcon(self.newPos);
                    moveNewPos = jtraj(currentJoints, newJoints, 30);



                case '-x'


                    self.newPos = self.currentPos.T;


                    self.newPos(1, 4) = self.newPos(1, 4) - self.stepSize;
                    newJoints = self.treebot.model.ikcon(self.newPos);
                    moveNewPos = jtraj(currentJoints, newJoints, 30);


                case 'y'

                    self.newPos = self.currentPos.T;

                    
                    self.newPos(2, 4) = self.newPos(2, 4) + self.stepSize;
                    newJoints = self.treebot.model.ikcon(self.newPos);
                    moveNewPos = jtraj(currentJoints, newJoints, 30);



                case '-y'

                    self.newPos = self.currentPos.T;

                    
                    self.newPos(2, 4) = self.newPos(2, 4) - self.stepSize;
                    newJoints = self.treebot.model.ikcon(self.newPos);
                    moveNewPos = jtraj(currentJoints, newJoints, 30);


                case 'z'
                    self.newPos = self.currentPos.T;

                    
                    self.newPos(3, 4) = self.newPos(3, 4) + self.stepSize;
                    newJoints = self.treebot.model.ikcon(self.newPos);
                    moveNewPos = jtraj(currentJoints, newJoints, 30);



                case '-z'

                    self.newPos = self.currentPos.T;

                    
                    self.newPos(3, 4) = self.newPos(3, 4) - self.stepSize;
                    newJoints = self.treebot.model.ikcon(self.newPos);
                    moveNewPos = jtraj(currentJoints, newJoints, 30);

            end
            for step = 1:size(moveNewPos, 1)

                   self.treebot.model.animate(moveNewPos(step,:));
                   drawnow();
                   pause(0.05);
                   if self.EstopPressed
                       disp('E-stop has been pressed!');
                       return;
                   end

            end
            self.q = newJoints;
            self.currentPos = self.newPos;
            currentJoints = newJoints;
            self.updateSliders();
        end
    end
end
