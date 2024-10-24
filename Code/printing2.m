classdef printing2 < LinearDobot
    properties
        num_layers = 2;  % Reduced number of layers for faster execution
        fig_handle;       % Figure handle for unified plotting
        dobot;
        total_birdhouses = 2; % Number of birdhouses to print
        birdhouse_positions = [0.2, 0.6];  % X positions for each birdhouse
        pause_time = 0.005;  % Pause time for faster animation
        z_offset = 0;        % Z-offset for all birdhouses (flat on the surface)
        y_position = 0.2;    % Fixed Y-position where all birdhouses will be printed
    end

    methods
        function self = printing2()
            addpath('../LinearDobot');
            addpath('../Print Files');
            disp('Class initialized. Starting the print function...');
            clf;
            hold on;

            % Initialize the Dobot model and rotate it 180 degrees around Z-axis
       
            self.PlotAndColourRobot();
           % Correctly rotate robot 180 degrees around Z-axis

            % Load PLY and find vertices for the birdhouse once
            [vertex_matrix, ply_filename] = self.loadPLYandFindVertices('birdhouse.ply', self.num_layers);
            if isempty(vertex_matrix)
                disp('Error loading PLY file. Exiting...');
                return;
            end

            % Move robotic arm and print all birdhouses
            for i = 1:self.total_birdhouses
                fprintf('Printing birdhouse %d of %d...\n', i, self.total_birdhouses);

                % Move birdhouse position based on the current iteration
                x_position = self.birdhouse_positions(i);
                translated_vertices = self.translateVertices(vertex_matrix, x_position, self.y_position, self.z_offset);

                % Move and animate the robotic arm for this birdhouse
                self.moveRoboticArm(translated_vertices);

                % Clear traces and show final STL model for this birdhouse
                self.clearTracesAndShowSTL('birdhouse.stl', x_position);
            end

            % Return the robot to its home position (0, 0, 0)
            self.moveRobotHome();
            disp('Finished printing all birdhouses.');
        end

        % Function to load the PLY file and extract vertices
        function [vertex_matrix, ply_filename] = loadPLYandFindVertices(self, ply_filename, num_layers)
            try
                model = pcread(ply_filename);  % Load PLY file
                vertices = model.Location;  % Extract vertices

                % Organize vertices by layers
                z_min = min(vertices(:, 3));
                z_max = max(vertices(:, 3));
                layer_height = (z_max - z_min) / num_layers;

                vertex_matrix = cell(num_layers, 1);  % Initialize vertex matrix
                for layer = 1:num_layers
                    z_layer_min = z_min + (layer - 1) * layer_height;
                    z_layer_max = z_layer_min + layer_height;
                    in_layer = vertices(:, 3) >= z_layer_min & vertices(:, 3) < z_layer_max;
                    vertex_matrix{layer} = vertices(in_layer, :);
                end

                disp('PLY file loaded and vertices extracted successfully.');
            catch ME
                disp(['Error loading PLY file: ', ME.message]);
                vertex_matrix = {};  % Return empty in case of error
            end
        end

        % Function to move robotic arm and print
        function moveRoboticArm(self, vertex_matrix)
            disp('Starting robot movements...');
            num_layers = length(vertex_matrix);

            for layer = 1:num_layers
                points_in_layer = vertex_matrix{layer};
                for i = 1:size(points_in_layer, 1)-1
                    start_point = points_in_layer(i, :);
                    end_point = points_in_layer(i + 1, :);

                    % Avoid small movements
                    if norm(end_point - start_point) < 0.005
                        continue;
                    end

                    % Get inverse kinematics for the next point
                    try
                        q_next = self.model.ikine(transl(end_point), self.model.getpos(), 'mask', [1 1 1 0 0 0]);
                    catch
                        disp('IK failed, skipping this point.');
                        continue;
                    end

                    % Create trajectory and animate
                    try
                        q_traj = jtraj(self.model.getpos(), q_next, 10);  % Reduced points for speed
                        self.animateTrajectory(q_traj);
                    catch ME
                        disp(['Error during trajectory creation/animation: ', ME.message]);
                    end

                    % Simulate printing with a red trace line
                    plot3([start_point(1), end_point(1)], [start_point(2), end_point(2)], [start_point(3), end_point(3)], 'r', 'LineWidth', 2);
                    pause(self.pause_time);  % Shorter pause for faster printing
                end
            end
        end

        % Helper function for animating trajectory
        function animateTrajectory(self, q_traj)
            for i = 1:size(q_traj, 1)
                self.model.animate(q_traj(i, :));
                drawnow;
            end
        end

        % Function to clear traces and show the final STL model
        function clearTracesAndShowSTL(self, stl_filename, x_offset)
            disp('Clearing traces and displaying final model...');
            cla;  % Clear the figure of previous lines

            % Load the final STL model and display it
            [faces, vertices, ~] = stlread(stl_filename);
            vertices(:, 1) = vertices(:, 1) + x_offset;  % Translate by x_offset for the next birdhouse
            vertices(:, 2) = vertices(:, 2) + self.y_position;  % Fixed Y position (0.2)

            % Plot STL model
            trisurf(faces, vertices(:, 1), vertices(:, 2), vertices(:, 3), 'FaceColor', 'cyan', 'EdgeColor', 'none');
            camlight('headlight');
            lighting gouraud;
            title('Final 3D-Printed Object');
        end

        % Function to translate vertices for each birdhouse
        function translated_vertices = translateVertices(self, vertex_matrix, x_offset, y_offset, z_offset)
            translated_vertices = cell(size(vertex_matrix));
            for i = 1:length(vertex_matrix)
                translated_vertices{i} = vertex_matrix{i};
                translated_vertices{i}(:, 1) = vertex_matrix{i}(:, 1) + x_offset;
                translated_vertices{i}(:, 2) = vertex_matrix{i}(:, 2) + y_offset;
                translated_vertices{i}(:, 3) = vertex_matrix{i}(:, 3) + z_offset;
            end
        end

        % Function to move the robot to the home position (0, 0, 0)
        function moveRobotHome(self)
            disp('Moving robot back to home position (0, 0, 0)...');
            home_q = self.model.ikine(transl(0, 0, 0), self.model.getpos(), 'mask', [1 1 1 0 0 0]);
            q_traj_home = jtraj(self.model.getpos(), home_q, 15);  % Smooth trajectory to home
            self.animateTrajectory(q_traj_home);
        end
    end
end
