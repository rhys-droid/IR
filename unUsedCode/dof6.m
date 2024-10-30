classdef dof6 < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Property1
    end

      methods 
		function self = dof6()
			clf
			clc
			input('Press enter to begin')
			self.Robot1();
            % self.Robot2();
		end
	end

    
    methods(Static)

        function Robot1()
            clf;
            clc

            L1 = Link('d',0.3,'a',0,'alpha',pi/2,'qlim',[-2*pi 2*pi]);
            L2 = Link('d', 0, 'a', 0.4, 'alpha', 0, 'qlim', [-pi, pi]);
            L3 = Link('d', 0, 'a', 0.4, 'alpha', 0, 'qlim', [-pi/2, pi]);
            L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/4, 5*pi/4]);
            L5 = Link('d', 0.2, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi, pi]);
            L6 = Link('d', 0.2, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi, pi]);
            L7 = Link('d', 0.2, 'a', 0, 'alpha', 0, 'qlim', [-2*pi, 2*pi]);

            
            robot = SerialLink([L1, L2, L3, L4, L5, L6, L7], 'name',"robottree");      
            
            % Creates a vector of n joint angles at 0.
            q = zeros(1, robot.n);  
            

            % Set the size of the workspace when drawing the robot
            workspace = [-4 4 -4 4 -4 4];
            scale = 0.75;

            % Plot the robot
            %

            robot.teach(q);

            % 4.4 Get the current joint angles based on the position in the model
            q = robot.getpos();

            % 4.5 Get the joint limits
            robot.qlim 
        end

        function Robot2() %I chnaged values for visual purposes
            clf;
            clc

            % 4.1 and 4.2: Define the DH Parameters to create the Kinematic 
			% model
            L1 = Link('d',0.5273,'a',0,'alpha',-pi/2,'qlim',[-pi pi]) 
            L2 = Link('d',0,'a',0.612,'alpha',0,'qlim',[-pi pi]) 
            L3 = Link('d',0,'a',0.5723,'alpha',0,'qlim',[-pi pi])
            L4 = Link('d',0.563,'a',0,'alpha',-pi/2,'qlim',[-pi pi])
            L5 = Link('d',0.5157,'a',0,'alpha',pi/2,'qlim',[-pi pi])
            L6 = Link('d',0.592,'a',0,'alpha',0,'qlim',[-pi pi])


			% Generate the model
            robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot')          
            
            % Creates a vector of n joint angles at 0.
            q = zeros(1, robot.n);  
            

            % Set the size of the workspace when drawing the robot
            workspace = [-4 4 -4 4 -4 4];
            scale = 0.75;

            % Plot the robot
            %q = [-0.7506, 0.5895, -1.8286, 0.5971];
            robot.plot(q,'workspace',workspace,'scale',scale); 

            % 4.3 Manually play around with the robot
            robot.teach([-17*pi/90, -pi/4, 0, -pi/4, 0, 0]);

            % 4.4 Get the current joint angles based on the position in the model
            q = robot.getpos();

            % 4.5 Get the joint limits
            robot.qlim 
        end

               function Robot3() 
            clf;
            clc

            % 4.1 and 4.2: Define the DH Parameters to create the Kinematic 
			% model
            L1 = Link('d',0.96,'a',0,'alpha',pi/2,'qlim',[-pi pi]) 
            L2 = Link('d',0.0,'a',1.67,'alpha',0,'qlim',[-pi pi])

            L3 = Link('d',0.0,'a',0,'alpha',-pi/2,'qlim',[-pi pi]) %These two links create the joint with 2DOF i think
            L4 = Link('d',1.23,'a',0,'alpha',0,'qlim',[-pi pi])

            L5 = Link('d',0.0,'a',0,'alpha',pi/2,'qlim',[-pi pi]) 
            L6 = Link('d',1.23,'a',0,'alpha',0,'qlim',[-pi pi])


			% Generate the model
            robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot')          
            
            % Creates a vector of n joint angles at 0.
            q = zeros(1, robot.n);  
            

            % Set the size of the workspace when drawing the robot
            workspace = [-4 4 -4 4 -4 4];
            scale = 0.75;

            % Plot the robot
            %q = [-0.7506, 0.5895, -1.8286, 0.5971];
            robot.plot(q,'workspace',workspace,'scale',scale); 

            % 4.3 Manually play around with the robot
            robot.teach();

            % 4.4 Get the current joint angles based on the position in the model
            q = robot.getpos();

            % 4.5 Get the joint limits
            robot.qlim 
        end
    end
end