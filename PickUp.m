classdef PickUp < handle
    properties
        treebot;
        z_offset = 0.95;  % Default z-offset
    end

    methods
        function self = PickUp(treebot)
            self.treebot = treebot;
        end

        function pickUpBirdhouse(self, start_pos, target_position)
            % Move to the birdhouse start position
            fprintf('TreeBot: Moving to pick up birdhouse at position (%.2f, %.2f, %.2f)...\n', start_pos);
            self.moveToPosition([start_pos, self.z_offset]);

            % Move birdhouse to the target location
            fprintf('TreeBot: Transporting birdhouse to target location (%.2f, %.2f, %.2f)...\n', target_position);
            self.moveToPosition(target_position);

            % Return to home position
            self.returnHome();
        end

        function moveToPosition(self, position)
            try
                q_target = self.treebot.model.ikine(transl(position), self.treebot.model.getpos(), 'mask', [1 1 1 0 0 0]);
                q_traj = jtraj(self.treebot.model.getpos(), q_target, 20); 
                self.animateTrajectory(q_traj);
            catch ME
                warning(['IK failed for position: ', mat2str(position), '. Error: ', ME.message]);
            end
        end

        function returnHome(self)
            q_traj_home = jtraj(self.treebot.model.getpos(), zeros(1, self.treebot.model.n), 20);
            self.animateTrajectory(q_traj_home);
            disp('TreeBot: Returned to home position.');
        end

        function animateTrajectory(self, q_traj)
            for i = 1:size(q_traj, 1)
                self.treebot.model.animate(q_traj(i, :));
                drawnow;
            end
        end
    end
end
