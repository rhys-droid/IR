classdef printing3 < LinearDobot
    properties
        num_layers = 2;  % Reduced number of layers for faster execution
        fig_handle;       % Figure handle for unified plotting
        dobot;
        birdhouse_positions = [0.2, 0.2; 0.6 0.2];  % X positions for each birdhouse
      
        pause_time = 0.001;  % Shorter pause time for faster animation
        z_offset = 0;        % Z-offset for all birdhouses (flat on the surface)
        y_position = 0.2;    % Fixed Y-position where all birdhouses will be printed
        trace_handles = [];
    end

    methods
        function self = printing3()
            addpath('../LinearDobot');
            addpath('../Print Files');
            disp('Class initialized. Starting the print function...');
            clf;
            hold on;

            self.PlotAndColourRobot();
            [vertex_matrix, ply_filename] = self.loadPLYandFindVertices('birdhouse.ply', self.num_layers);

            for i = 1:2
                fprintf('Printing birdhouse %d of %d...\n', i, 2);

             
                x_position = self.birdhouse_positions(i, 1);
                y_position = self.birdhouse_positions(i, 2);
                translated_vertices = self.translateVertices(vertex_matrix, x_position, y_position, self.z_offset);

            
                self.moveRoboticArm(translated_vertices);

            
                self.clearTracesAndShowPLY('birdhouse.ply', [x_position, y_position]);
            end

           
            self.moveRobotHome();
            disp('Finished printing all birdhouses.');
        end
        
        function [vertex_matrix, ply_filename] = loadPLYandFindVertices(self, ply_filename, num_layers)
                model = pcread(ply_filename);  
                vertices = model.Location; 

      
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
        end

     
        function moveRoboticArm(self, vertex_matrix)
            disp('Starting robot movements...');
            num_layers = length(vertex_matrix);
            self.trace_handles = [];

            for layer = 1:num_layers
                points_in_layer = vertex_matrix{layer};
                for i = 1:size(points_in_layer, 1) - 1
                    start_point = points_in_layer(i, :);
                    end_point = points_in_layer(i + 1, :);

                    if norm(end_point - start_point) < 0.005
                        continue;
                    end

                    try
                        q_next = self.model.ikine(transl(end_point), self.model.getpos(), 'mask', [1 1 1 0 0 0]);
                    catch
                        disp('IK failed, skipping this point.');
                        continue;
                    end

                    % Create trajectory and animate
                    try
                        q_traj = jtraj(self.model.getpos(), q_next, 5);  % Fewer points for faster execution
                        self.animateTrajectory(q_traj);
                    catch ME
                        disp(['Error during trajectory creation/animation: ', ME.message]);
                    end

                    % Simulate printing with a red trace line
                    h2 = plot3([start_point(1), end_point(1)], [start_point(2), end_point(2)], [start_point(3), end_point(3)], 'r', 'LineWidth', 2);
                    display(h2);
                    self.trace_handles = [self.trace_handles, h2];
                    pause(self.pause_time);  % Shorter pause for faster printing
                end
            end
            disp('Finished all layers.');
        end

        function animateTrajectory(self, q_traj)
            for i = 1:size(q_traj, 1)
                self.model.animate(q_traj(i, :));
                drawnow;
            end
        end

     
        function clearTracesAndShowPLY(self, ply_filename, position)
            disp('Clearing traces and displaying final model...');

            % Delete all stored trace handles
            for i = 1:length(self.trace_handles)
                delete(self.trace_handles(i));
            end
            self.trace_handles = [];  % Clear the handles array

            % Load and display the PLY model
            [faceData, vertexData, ~] = plyread(ply_filename);
            trisurf(faceData, vertexData(:, 1) + position(1), vertexData(:, 2) + position(2), vertexData(:, 3), 'FaceColor', 'magenta');
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
            q_traj_home = jtraj(self.model.getpos(), home_q, 5);  % Faster trajectory to home
            self.animateTrajectory(q_traj_home);
        end
    end
end
