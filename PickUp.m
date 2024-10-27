classdef PickUp < handle
    properties
        treebot;  % TreeBot instance
        print_positions;  % Array of print positions for birdhouses
        target_position = [1.5, 0.5, 1.0];  % Destination for birdhouses
        z_offset = 0.95;  % Height offset for pick-up
        pause_time = 1;  % Pause time to simulate grasping
    end

    methods
        function self = PickUp(treebot)
            % Initialize the PickUp class with TreeBot instance
            self.treebot = treebot;
            self.print_positions = [-0.2, -0.8, self.z_offset; -0.2, -1.2, self.z_offset];
        end

        function startPickUp(self)
            % Loop through each printed birdhouse position and pick it up
            for i = 1:size(self.print_positions, 1)
                fprintf('TreeBot: Moving to pick up birdhouse %d...\n', i);
                self.moveToPosition(self.print_positions(i, :));

                % Simulate grasping
                disp('TreeBot: Grasping birdhouse...');
                pause(self.pause_time);

                % Move birdhouse to target location
                fprintf('TreeBot: Moving birdhouse %d to target location...\n', i);
                self.moveToPosition(self.target_position);

                % Simulate releasing the birdhouse
                disp('TreeBot: Releasing birdhouse at target location.');
                pause(self.pause_time);

                % Return TreeBot to its home position
                self.returnHome();
            end
        end

        function moveToPosition(self, position)
            % Move TreeBot to a specified XYZ position
            try
                q_target = self.treebot.model.ikine(transl(position), self.treebot.model.getpos(), 'mask', [1 1 1 0 0 0]);
                q_traj = jtraj(self.treebot.model.getpos(), q_target, 20);  % Smooth trajectory
                self.animateTrajectory(q_traj);
            catch ME
                warning(['IK failed for position: ', mat2str(position), '. Error: ', ME.message]);
            end
        end

        function returnHome(self)
            % Return TreeBot to its initial home position
            q_traj_home = jtraj(self.treebot.model.getpos(), zeros(1, self.treebot.model.n), 20);
            self.animateTrajectory(q_traj_home);
            disp('TreeBot: Returned to home position.');
        end

        function animateTrajectory(self, q_traj)
            % Helper function to animate the trajectory
            for i = 1:size(q_traj, 1)
                self.treebot.model.animate(q_traj(i, :));
                drawnow;
            end
        end
    end
end
