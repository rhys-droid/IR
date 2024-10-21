classdef LinearDobot2 < RobotBaseClass
    %% DobotMagician
    % This class is based on the DobotMagician. 
    % URL: https://en.dobot.cn/products/education/magician.html
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarantee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)
        plyFileNameStem = 'LinearDobot';
        %> defaultRealQ 
        defaultRealQ = [0, pi/4, pi/4, 0, 0];
    end

    methods (Access = public)
        %% Constructor
        function self = LinearDobot2(baseTr)
            % Error handling during model creation and initialization
            try
                disp('Creating LinearDobot...');
                self.CreateModel();
                
                if nargin < 1
                    baseTr = eye(4);  % Identity matrix for default base
                end
                
                % Applying base transformations
                self.model.base = baseTr * trotx(pi/2) * troty(pi/2);
                
                % Plot and color the robot for initial visualization
                self.PlotAndColourRobot();
            catch ME
                disp(['Error during LinearDobot initialization: ', ME.message]);
            end
        end

        %% Create Model with Debugging
        function CreateModel(self)
            % Debugging DH parameters and joint limits
            disp('Creating the robot model with the following DH parameters and joint limits:');
            
            link(1) = Link([pi 0 0 pi/2 1]);  % PRISMATIC Link
            link(2) = Link('d', 0.103 + 0.0362, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'qlim', [deg2rad(-135), deg2rad(135)]);
            link(3) = Link('d', 0, 'a', 0.135, 'alpha', 0, 'offset', -pi/2, 'qlim', [deg2rad(5), deg2rad(80)]);
            link(4) = Link('d', 0, 'a', 0.147, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-5), deg2rad(85)]);
            link(5) = Link('d', 0, 'a', 0.06, 'alpha', pi/2, 'offset', 0, 'qlim', [deg2rad(-180), deg2rad(180)]);
            link(6) = Link('d', -0.05, 'a', 0, 'alpha', 0, 'offset', 0, 'qlim', [deg2rad(-85), deg2rad(85)]);

            % Set up prismatic joint limits for Link 1
            link(1).qlim = [0 0.8];

            % Printing DH parameters and joint limits for each link
            for i = 1:length(link)
                fprintf('Link %d DH Parameters: d = %.4f, a = %.4f, alpha = %.4f, offset = %.4f\n', ...
                    i, link(i).d, link(i).a, link(i).alpha, link(i).offset);
                fprintf('Link %d Joint Limits: [%f, %f]\n', i, link(i).qlim(1), link(i).qlim(2));
            end
            
            % Create the robot model
            self.model = SerialLink(link, 'name', self.plyFileNameStem);
        end

        %% Custom Plot and Color Function
        function PlotAndColourRobot(self)
            % Plot and color the robot
            q0 = zeros(1, self.model.n);  % Neutral position for the robot
            self.model.plot(q0, 'workspace', [-1 1 -1 1 -1 1], 'noarrow', 'nobase');
            camlight('headlight');
            lighting gouraud;
            disp('Robot plotted successfully.');
        end

        %% Function to Test the Robot's Movements (optional)
        function TestMoveDobotMagician(self)
            % Test movement for the robot with trajectory generation
            qPath = jtraj(self.model.qlim(:,1)', self.model.qlim(:,2)', 50);                       
            for i = 1:50
                self.model.animate(qPath(i,:));
                pause(0.2);
            end
        end
    end
end
