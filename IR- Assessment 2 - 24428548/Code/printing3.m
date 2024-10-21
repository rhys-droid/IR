classdef printing3 < LinearDobot
    properties
        num_layers = 2;  % Reduced number of layers for faster execution
        fig_handle;       % Figure handle for unified plotting
        dobot;
        total_birdhouses = 6;  % Total number of birdhouses to print
        birdhouse_gap_x = 0.4; % Gap between birdhouses along the x-axis
        birdhouse_y = 0.2;     % Y position for all birdhouses
    end

    methods
        function self = printing3()
            addpath('../LinearDobot');
            addpath('../Print Files');
            disp('Class initialized. Starting the print function...');
            clf;
            hold on;

            % Initialize the Dobot model
            self.PlotAndColourRobot();

            % Load PLY and find vertices
            [vertex_matrix, ply_filename] = self.loadPLYandFindVertices('birdhouse.ply', self.num_layers);

            if isempty(vertex_matrix)
                disp('Error loading PLY file. Exiting...');
                return;
            end

            % Move robotic arm and print all birdhouses
            for i = 1:self.total_birdhouses
                fprintf('Printing birdhouse %d of %d...\n', i, self.total_birdhouses);

                % Set the x-offset for each birdhouse
                x_offset = 0.2 + (i - 1) * self.birdhouse_gap_x;
                
                % Translate vertices and set the fixed y value
                translated_vertices = self.translateVertices(vertex_matrix, x_offset, self.birdhouse_y);
                
                % Move and animate the robotic arm for this birdhouse
                self.moveRoboticArm(translated_vertices);

                % Clear traces and show final PLY model for this birdhouse
                self.clearTracesAndShowPLY(ply_filename, x_offset);
            end

            disp('Finished printing all birdhouses.');
        end

        % Function to load the PLY file and extract vertices
        function [vertex_matrix, ply_filename] = loadPLYandFindVertices(self, ply_filename, num_layers)
            try
                model = pcread(ply_filename);  % Load PLY file
                vertices = model.Location;  % Extract vertices

                % Organize vertices by layers
                z_min = min(vertices(:,3));
                z_max = max(vertices(:,3));
                layer_height = (z_max - z_min) / num_layers;

                vertex_matrix = cell(num_layers, 1);  % Initialize vertex matrix
                for layer = 1:num_layers
                    z_layer_min = z_min + (layer - 1) * layer_height;
                    z_layer_max = z_layer_min + layer_height;
                    in_layer = vertices(:,3) >= z_layer_min & vertices(:,3) < z_layer_max;
                    vertex_matrix{layer} = vertices(in_layer, :);
                end

                disp('PLY file loaded, vertices translated, and extracted successfully.');
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
                        q_traj = jtraj(self.model.getpos(), q_next, 10); % Reduced points for speed
                        self.animateTrajectory(q_traj);
                    catch ME
                        disp(['Error during trajectory creation/animation: ', ME.message]);
                    end

                    % Simulate printing with a red trace line
                    plot3([start_point(1), end_point(1)], [start_point(2), end_point(2)], [start_point(3), end_point(3)], 'r', 'LineWidth', 2);
                    pause(0.01);  % Shorter pause for faster printing
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

        % Function to clear traces and show the final PLY model
        function clearTracesAndShowPLY(self, ply_filename, x_offset)
            disp('Clearing traces and displaying final model...');
            cla;  % Clear the figure of previous lines

            % Load the final PLY model and display it
            model = pcread(ply_filename);
            vertices = model.Location;
            vertices(:, 1) = vertices(:, 1) + x_offset;  % Translate by x_offset for the next birdhouse
            vertices(:, 2) = vertices(:, 2) + self.birdhouse_y;  % Set y offset to 0.2
            plot3(vertices(:,1), vertices(:,2), vertices(:,3), 'bo', 'MarkerSize', 5);  
            camlight('headlight');
            lighting gouraud;
            title('Final 3D-Printed Object');
        end

        % Function to translate vertices for each birdhouse
        function translated_vertices = translateVertices(self, vertex_matrix, x_offset, y_offset)
            translated_vertices = cell(size(vertex_matrix));
            for i = 1:length(vertex_matrix)
                translated_vertices{i} = vertex_matrix{i};
                translated_vertices{i}(:, 1) = vertex_matrix{i}(:, 1) + x_offset;  % Translate along x-axis
                translated_vertices{i}(:, 2) = y_offset;  % Set y to a fixed value (0.2)
            end
        end
    end
end
