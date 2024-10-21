classdef printing2 < LinearDobot
    properties
        num_layers = 20;  % Number of layers to simulate
        fig_handle;       % Figure handle for unified plotting
        dobot;
    end

    methods
        function self = printing2()
            addpath('../LinearDobot');
            addpath('../Print Files');
            disp('Class initialized. Starting the print function...');
            clf;
            % Start by setting the environment and performing the task
            self.dobot = LinearDobot();
          
            self.setEnvironment();
              hold on;
            self.executePrintTask();
        end

        function setEnvironment(self)
            %% Step 1: Initialize the Dobot
            disp('Initializing Dobot...');
          


            % Set prismatic joint limits if not already set
            try
                if isempty(self.dobot.model.links(1).qlim)
                    self.dobot.model.links(1).qlim = [0.1 1];  % Set limits for the prismatic joint
                    disp('Prismatic joint limits set successfully.');
                end
            catch ME
                disp(['Error setting prismatic joint limits: ', ME.message]);
                return;
            end

            %% Step 2: Plot the robot in the initial position
            disp('Plotting the Dobot robot in the initial position...');
            try
q0 = zeros(1, self.dobot.model.n);  % Set all joint angles to zero
                self.fig_handle = figure(1); clf;  % Clear figure and reset
                hold on;
                grid on;
                axis equal;
                self.dobot.model.plot(q0, 'workspace', [-1 1 -1 1 -0.1 1], 'nobase', 'noarrow');
                disp('Robot successfully plotted.');
            catch ME
                disp(['Error plotting the robot: ', ME.message]);
                return;
            end
        end

        function executePrintTask(self)
            %% Step 3: Load the PLY file and organize vertex points by layers
            try
                model = pcread('birdhouse.ply');  % Ensure birdhouse.ply is in Print Files folder
                disp('PLY file loaded successfully.');
                vertices = model.Location;  % Extract vertices (points)
            catch ME
                disp(['Could not load the birdhouse.ply file. Error: ', ME.message]);
                return;
            end

            %% Step 4: Sort vertex points into layers
            z_min = min(vertices(:,3));
            z_max = max(vertices(:,3));
            layer_height = (z_max - z_min) / self.num_layers;

            vertex_matrix = cell(self.num_layers, 1);  % Each cell contains points in that layer
            for layer = 1:self.num_layers
                z_layer_min = z_min + (layer - 1) * layer_height;
                z_layer_max = z_layer_min + layer_height;
                in_layer = vertices(:,3) >= z_layer_min & vertices(:,3) < z_layer_max;
                vertex_matrix{layer} = vertices(in_layer, :);
            end

            %% Step 5: Move the robot to the first point
            disp('Moving to the first vertex...');
            try
                q0 = double(self.dobot.model.getpos());  % Ensure the robot's joint positions are of type double
                first_point = vertex_matrix{1}(1, :);  % First point in the first layer
                q_first = self.dobot.model.ikine(transl(first_point), q0, 'mask', [1 1 1 0 0 0]);  % Solve IK for the first point
            catch ME
                disp(['Error getting robot position: ', ME.message]);
                return;
            end

            % Trajectory to the first point
            q_traj = jtraj(q0, q_first, 50);  % Interpolate a smooth trajectory
            self.animateTrajectory(q_traj);  % Animate the movement

            %% Step 6: Traverse all points and simulate printing
            disp('Starting robot movements to simulate printing...');
            for layer = 1:self.num_layers
                points_in_layer = vertex_matrix{layer};
                for i = 1:size(points_in_layer, 1)-1
                    start_point = points_in_layer(i, :);
                    end_point = points_in_layer(i + 1, :);

                    % Calculate inverse kinematics for the next point
                    q_next = self.dobot.model.ikine(transl(end_point), double(self.dobot.model.getpos()), 'mask', [1 1 1 0 0 0]);
                    
                    % Create trajectory for smooth movement
                    q_traj = jtraj(self.dobot.model.getpos(), q_next, 50);
                    self.animateTrajectory(q_traj);  % Animate the robot to the next point
                    
                    % Simulate printing by drawing a trace between points
                    plot3([start_point(1), end_point(1)], [start_point(2), end_point(2)], [start_point(3), end_point(3)], 'r', 'LineWidth', 2);
                    pause(0.05);  % Simulate time for movement
                end
            end

            %% Step 7: Clear traces and visualize the final birdhouse object
            disp('Printing complete. Displaying final model...');
            pause(1);
            cla;  % Clear all previous traces
            plot3(vertices(:,1), vertices(:,2), vertices(:,3), 'bo', 'MarkerSize', 5);  % Display the final birdhouse
            camlight('headlight');
            lighting gouraud;
            axis equal;
            title('Final 3D-Printed Birdhouse');
        end

        function animateTrajectory(self, q_traj)
            % Animates a smooth trajectory for the robot
            for i = 1:size(q_traj, 1)
                self.dobot.model.animate(q_traj(i, :));  % Standard animation call
                drawnow;
            end
        end
    end
end
