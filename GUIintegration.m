classdef GUIintegration < handle
    properties
        robot;  % TreeBot object
        app;    % GUI app object
        q;      % Joint angles
    end

    methods
        % Constructor
        function self = GUIintegration()
            clf;  % Clear the current figure

            self.q = zeros(1, 7);  % Initialize joint angles to 0

            self.runRobot();  % Initialize the robot
            self.openGUI();   % Open the GUI
        end

        % Initialize the TreeBot robot
        function runRobot(self)
            addpath('TreeBot');  % Add the robot folder to path

            % Create the robot object
            self.robot = TreeBot();

            % Plot the robot with the initial configuration
            self.robot.PlotAndColourRobot();
            self.robot.AnimateRobot(self.q);  % Plot the robot at initial pose

            % Ensure the sliders reflect the robot's initial pose
            self.updateSliders();
        end

        % Open the GUI and set up the callbacks
        function openGUI(self)
            self.app = GUI();  % Create the GUI object

            % Set slider callbacks to update joint angles
            self.app.Link1Slider.ValueChangingFcn = @(src, event) self.updateJoint(1, event.Value);
            self.app.Link2Slider.ValueChangingFcn = @(src, event) self.updateJoint(2, event.Value);
            self.app.Link3Slider.ValueChangingFcn = @(src, event) self.updateJoint(3, event.Value);
            self.app.Link4Slider.ValueChangingFcn = @(src, event) self.updateJoint(4, event.Value);
            self.app.Link5Slider.ValueChangingFcn = @(src, event) self.updateJoint(5, event.Value);
            self.app.Link6Slider.ValueChangingFcn = @(src, event) self.updateJoint(6, event.Value);
        end

        % Update the corresponding joint angle when the slider is moved
        function updateJoint(self, jointIndex, value)
            % Update the joint angle in radians
            self.q(jointIndex) = deg2rad(value);

            % Use animate to smoothly update the robot configuration
            self.robot.AnimateRobot(self.q);
        end

        % Method to update the slider values based on robot state
        function updateSliders(self)
            % Convert the current joint angles from radians to degrees
            qDegrees = rad2deg(self.q);

            % Update each slider with the current joint angle
            self.app.Link1Slider.Value = qDegrees(1);
            self.app.Link2Slider.Value = qDegrees(2);
            self.app.Link3Slider.Value = qDegrees(3);
            self.app.Link4Slider.Value = qDegrees(4);
            self.app.Link5Slider.Value = qDegrees(5);
            self.app.Link6Slider.Value = qDegrees(6);
        end
    end
end
