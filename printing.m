classdef printing < handle
    properties
        num_layers = 2;
        fig_handle;
        dobot;   
        birdhouse_positions = [-0.2, -0.8; -0.2 -1.2];
        pause_time = 0.001;
        z_offset = 0.95;
        trace_handles = [];
    end

    methods
        function self = printing(dobot)
            self.dobot = dobot;  
            addpath('../Print Files');
            disp('Class initialized. Starting the print function...');
        end

        function printBirdhouse(self, i)
            [vertex_matrix, ply_filename] = self.loadPLYandFindVertices('birdhouse.ply', self.num_layers);
            fprintf('Printing birdhouse %d of %d...\n', i, 2);
            x_position = self.birdhouse_positions(i, 1);
            y_position = self.birdhouse_positions(i, 2);
            translated_vertices = self.translateVertices(vertex_matrix, x_position, y_position, self.z_offset);
            self.moveRoboticArm(translated_vertices);
            self.clearTracesAndShowPLY('birdhouse.ply', [x_position, y_position]);
            fprintf('Birdhouse %d ready for pickup at position (%0.2f, %0.2f).\n', x_position, y_position);
        end

        function [vertex_matrix, ply_filename] = loadPLYandFindVertices(self, ply_filename, num_layers)
                model = pcread(ply_filename);  
                vertices = model.Location; 

                min_points_per_layer = 5;
                z_min = min(vertices(:, 3));
                z_max = max(vertices(:, 3));
                layer_height = (z_max - z_min) / num_layers;

                vertex_matrix = cell(num_layers, 1); 
        for layer = 1:num_layers
            z_layer_min = z_min + (layer - 1) * layer_height;
            z_layer_max = z_layer_min + layer_height;
    
           
            in_layer = vertices(:, 3) >= z_layer_min & vertices(:, 3) < z_layer_max;
            points_in_layer = vertices(in_layer, :);
    
          
            if size(points_in_layer, 1) < min_points_per_layer
                warning('Not enough points in layer %d, skipping...', layer);
                continue;
            end
    
            vertex_matrix{layer} = points_in_layer;
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
                    % start_point(3) = start_point(3) + self.z_offset;
                    % end_point(3) = end_point(3) + self.z_offset;

                    if norm(end_point - start_point) < 0.1
                        continue;
                    end
                  
                

                    % 
                    % J = self.dobot.model.jacob0(self.dobot.model.getpos());
                    % J_trans = J(1:3, :);
                    % joint_velocities = pinv(J_trans) * (end_point - start_point)/norm(end_point - start_point);
                    % 
                    % q_current = self.dobot.model.getpos();
                    % q_next = q_current + joint_velocities' * self.pause_time;
                    % self.animateTrajectory(q_next);
                    % 
                    % 
                    try
                        q_next = self.dobot.model.ikine(transl(end_point), self.dobot.model.getpos(), 'mask', [1 1 1 0 0 0]);
                    catch
                        disp('IK failed, skipping this point.');
                        continue;
                    end

                    try
                        q_traj = jtraj(self.dobot.model.getpos(), q_next, 10);  % Fewer points for faster execution
                        self.animateTrajectory(q_traj);
                    catch ME
                        disp(['Error during trajectory creation/animation: ', ME.message]);
                    end

                 
                    h2 = plot3([start_point(1), end_point(1)], [start_point(2), end_point(2)], [start_point(3), end_point(3)], 'r', 'LineWidth', 2);
                    % display(h2);
                    self.trace_handles = [self.trace_handles, h2];
             
                end
            end
            disp('Finished all layers.');
        end

        function animateTrajectory(self, q_traj)
            for i = 1:size(q_traj, 1)
                self.dobot.model.animate(q_traj(i, :));
                drawnow;
            end
        end

     
        function clearTracesAndShowPLY(self, ply_filename, position)
            disp('Clearing traces and displaying final model...');

    
            for i = 1:length(self.trace_handles)
                delete(self.trace_handles(i));
            end
            self.trace_handles = []; 

            [faceData, vertexData, ~] = plyread(ply_filename);
            Rz = [cosd(90), -sind(90), 0; sind(90), cosd(90), 0; 0, 0, 1];

            vertexData = (Rz * vertexData')';
            trisurf(faceData, vertexData(:, 1) + position(1), vertexData(:, 2) + position(2), vertexData(:, 3) + 0.925, 'FaceColor', 'magenta');
        end


        function translated_vertices = translateVertices(self, vertex_matrix, x_offset, y_offset, z_offset)
            translated_vertices = cell(size(vertex_matrix));

            Rz = [cosd(90), -sind(90), 0; sind(90), cosd(90), 0; 0, 0, 1];

            for i = 1:length(vertex_matrix)
                rotated_matrix = (Rz * vertex_matrix{i}')';
                translated_vertices{i} = rotated_matrix;
                translated_vertices{i}(:, 1) = rotated_matrix(:, 1) + x_offset;
                translated_vertices{i}(:, 2) = rotated_matrix(:, 2) + y_offset;
                translated_vertices{i}(:, 3) = rotated_matrix(:, 3) + z_offset;
            end
        end

       
        function moveRobotHome(self)
            disp('Moving robot back to home position (0, 0, 0)...');
            try
            home_q = self.dobot.model.ikunc(transl(0, 0, self.z_offset), self.dobot.model.getpos(), 'mask', [1 1 1 0 0 0], 'tol', 1e-3, 'ilimit', 1000);
            q_traj_home = jtraj(self.dobot.model.getpos(), home_q, 3);  
            self.animateTrajectory(q_traj_home);
            catch 
                disp('uh oh');
        end

        end

        function birdhouse_position = birdhouseNumber(self, x_position)
            [~, birdhouse_index] = min(abs(self.birdhouse_positions(:, 1) - x_position));
            birdhouse_position = self.birdhouse_positions(birdhouse_index, :);
        end
    end
end
