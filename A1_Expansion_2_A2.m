classdef A1_Expansion_2_A2 < DobotMagician
    properties 
        brickHandle
    end

    methods
        function self = A1_Expansion_2_A2()
            clf;
            input('Press enter to begin');
            self.setEnvironment();
            self.executeTask();
        end

        function setEnvironment(self)
            clf;
            hold on;
            self.model.base = troty(pi/2) * trotz(pi/2) * transl(0, 0.5, 0.65);
            self.PlotAndColourRobot();
            self.AttachGripper();
            self.spawnEnvironment();
        end

        function spawnEnvironment(self)
            surf([-2.5, -2.5; 2.5, 2.5], [-2.5, 2.5; -2.5, 2.5], [0, 0; 0, 0], 'CData', imread('Floor.jpg'), 'FaceColor', 'texturemap');
            surf([-2.5, 2.5; -2.5, 2.5], [2.5, 2.5; 2.5, 2.5], [0, 0; 2.5, 2.5], 'CData', imread('Frontwall.jpg'), 'FaceColor', 'texturemap');
            surf([-2.5, -2.5; -2.5, -2.5], [-2.5, 2.5; -2.5, 2.5], [0, 0; 2.5, 2.5], 'CData', imread('RSidewall.jpg'), 'FaceColor', 'texturemap');
            surf([2.5, 2.5; 2.5, 2.5], [-2.5, 2.5; -2.5, 2.5], [0, 0; 2.5, 2.5], 'CData', imread('LSidewall.jpg'), 'FaceColor', 'texturemap');
            [faceData, vertexData, ~] = plyread('tableBrown2.1x1.4x0.5m.ply', 'tri');
            trisurf(faceData, vertexData(:, 1), vertexData(:, 2), vertexData(:, 3), 'FaceColor', 'magenta');
            [faceData, vertexData, ~] = plyread('bookcaseTwoShelves0.5x0.2x0.5m.ply', 'tri');
            shelfPosition = transl(-0.5, -0.25, 0.5) * trotx(-pi/2);
            transformedVertices = (shelfPosition * [vertexData, ones(size(vertexData, 1), 1)]')';
            trisurf(faceData, transformedVertices(:, 1), transformedVertices(:, 2), transformedVertices(:, 3), 'FaceColor', 'blue');
            [faceData, vertexData, ~] = plyread('HalfSizedRedGreenBrick.ply', 'tri');
            self.brickHandle{1} = trisurf(faceData, vertexData(:, 1) - 0.7 + 0.1, vertexData(:, 2) + 0.125, vertexData(:, 3) + 0.62, 'FaceColor', 'red');
            for i = 2:5
                self.brickHandle{i} = trisurf(faceData, vertexData(:, 1) - 0.7 + 0.1 * (i-1), vertexData(:, 2) - 0.125, vertexData(:, 3) + 0.55, 'FaceColor', 'red');
            end
            for j = 6:9
                self.brickHandle{j} = trisurf(faceData, vertexData(:, 1) - 0.7 + 0.1 * (j-5), vertexData(:, 2) + 0.125, vertexData(:, 3) + 0.55, 'FaceColor', 'red');
            end
            [faceData, vertexData, ~] = plyread('fenceFinal.ply', 'tri');
            fencePosition = transl(1.2, -1, 0.0);
            transformedVertices = (fencePosition * [vertexData, ones(size(vertexData, 1), 1)]')';
            trisurf(faceData, transformedVertices(:, 1), transformedVertices(:, 2), transformedVertices(:, 3), 'FaceColor', 'yellow');
            trisurf(faceData, transformedVertices(:, 1), transformedVertices(:, 2) + 1, transformedVertices(:, 3), 'FaceColor', 'yellow');
            trisurf(faceData, transformedVertices(:, 1), transformedVertices(:, 2) + 2, transformedVertices(:, 3), 'FaceColor', 'yellow');
            [faceData, vertexData, ~] = plyread('personMaleCasual.ply', 'tri');
            personPosition = transl(1.8, -0.5, 0);
            transformedVertices = (personPosition * [vertexData, ones(size(vertexData, 1), 1)]')';
            trisurf(faceData, transformedVertices(:, 1), transformedVertices(:, 2), transformedVertices(:, 3), 'FaceColor', 'blue');
            [faceData, vertexData, ~] = plyread('fireExtinguisher.ply', 'tri');
            extinguisherPosition = transl(2, -1.5, 0.0);
            transformedVertices = (extinguisherPosition * [vertexData, ones(size(vertexData, 1), 1)]')';
            trisurf(faceData, transformedVertices(:, 1), transformedVertices(:, 2), transformedVertices(:, 3), 'FaceColor', 'red');
            [faceData, vertexData, ~] = plyread('emergencyStopWallMounted.ply', 'tri');
            stopButtonPosition = transl(2.4, -0.75, 1);
            transformedVertices = (stopButtonPosition * [vertexData, ones(size(vertexData, 1), 1)]')';
            trisurf(faceData, transformedVertices(:, 1), transformedVertices(:, 2), transformedVertices(:, 3), 'FaceColor', 'red');
        end

        function executeTask(self)
            brick_positions = self.getBrickPositions();
            placement_positions = self.getPlacementPositions();
            totalBricks = length(brick_positions);
            for i = 1:totalBricks
                percentComplete = (i / totalBricks) * 100;
                fprintf('Processing brick %d of %d (%.2f%% complete)\n', i, totalBricks, percentComplete);
                self.moveToPosition(brick_positions{i}, 0.1);
                self.AttachGripper();
                self.moveToPosition(placement_positions{i}, 0.1);
                self.AttachGripper();
                self.transportBrick(i, placement_positions{i});
            end
        end

        function positions = getBrickPositions(self)
            positions = {
                transl(-0.6, 0.125, 0.62);
                transl(-0.7, -0.125, 0.55);
                transl(-0.6, -0.125, 0.55);
                transl(-0.5, -0.125, 0.55);
                transl(-0.4, -0.125, 0.55);
                transl(-0.7, 0.125, 0.55);
                transl(-0.6, 0.125, 0.55);
                transl(-0.5, 0.125, 0.55);
                transl(-0.4, 0.125, 0.55);
            };
        end

        function positions = getPlacementPositions(self)
            positions = {
                transl(0.5, 0.5, 0.55);
                transl(0.62, 0.5, 0.55);
                transl(0.74, 0.5, 0.55);
                transl(0.5, 0.5, 0.62);
                transl(0.62, 0.5, 0.62);
                transl(0.74, 0.5, 0.62);
                transl(0.5, 0.5, 0.69);
                transl(0.62, 0.5, 0.69);
                transl(0.74, 0.5, 0.69);
            };
        end

        function transportBrick(self, brickIndex, endPosition)
            brickHandle = self.brickHandle{brickIndex};
            brickVertices = get(brickHandle, 'Vertices');
            meanPosition = mean(brickVertices, 1);
            currentPosition = meanPosition(1:3);
            targetPosition = endPosition(1:3, 4)';
            translation = targetPosition - currentPosition;
            updatedVertices = bsxfun(@plus, brickVertices, translation);
            set(brickHandle, 'Vertices', updatedVertices);
            drawnow;
        end

        function moveToPosition(self, targetPosition, hoverHeight)
            if nargin < 3
                hoverHeight = 0.1;
            end
            hoverPosition = targetPosition;
            hoverPosition(3, 4) = hoverPosition(3, 4) + hoverHeight;
            q_hover = self.model.ikcon(hoverPosition);
            q_traj_hover = jtraj(self.model.getpos(), q_hover, 100);
            for j = 1:size(q_traj_hover, 1)
                self.model.animate(q_traj_hover(j, :));
                drawnow;
            end
            actualHoverPose = self.model.fkine(self.model.getpos()).t;
            hoverPositionDifference = norm(actualHoverPose - hoverPosition(1:3, 4));
            targetPose = targetPosition;
            q_target = self.model.ikcon(targetPose);
            q_traj = jtraj(self.model.getpos(), q_target, 100);
            for j = 1:size(q_traj, 1)
                self.model.animate(q_traj(j, :));
                drawnow;
            end
            actualPose = self.model.fkine(self.model.getpos()).t;
            targetPosition3x1 = targetPose(1:3, 4);
            positionDifference = norm(actualPose - targetPosition3x1);
            if positionDifference <= 0.005
                disp('Position is within ±5mm tolerance.');
            else
                disp('Position is outside the ±5mm tolerance.');
            end
        end

        function workSpaceSize(self, num)
            workSpacePoints = zeros(num, 3);
            for i = 1:num
                qSample = self.model.qlim(:,1) + (self.model.qlim(:,2) - self.model.qlim(:,1)) .* rand(self.model.n, 1);
                Trans = self.model.fkine(qSample).T;
                workspacePoints(i, :) = Trans(1:3, 4)';    
            end
            wr = sqrt(sum(workSpacePoints.^2, 2));
            Wradius = max(wr);
            volume = convhulln(workspacePoints);
            disp(['Workspace Radius', num2str(Wradius), 'm', 'Volume', num2str(volume), 'm^3']);
        end
    end
end
