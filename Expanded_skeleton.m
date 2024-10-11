classdef Expanded_skeleton < handle
    properties 
        PrintArm
        AssemblyArm
        Extruder
        EnvironmentObjects
        WildlifeDetected
        TaskComplete
    end

    methods
        function self = Expanded_skeleton()
            clf;
            input('Press enter to begin');
            self.WildlifeDetected = false;
            self.TaskComplete = false;
            self.setEnvironment();
            self.initializeArms();
            self.executeConstruction();
        end

        function setEnvironment(self)
            hold on;
            axis equal;
            self.spawnEnvironment();
            self.setupSafetyFeatures();
        end

        function spawnEnvironment(self)
            self.EnvironmentObjects = {};
            surf([-2.5, -2.5; 2.5, 2.5], [-2.5, 2.5; -2.5, 2.5], [0, 0; 0, 0], 'CData', imread('Floor.jpg'), 'FaceColor', 'texturemap');
        end

        function setupSafetyFeatures(self)
            [faceData, vertexData, ~] = plyread('safetyBarrier.ply', 'tri');
            barrierPosition = transl(0, 0, 0);
            transformedVertices = (barrierPosition * [vertexData, ones(size(vertexData, 1), 1)]')';
            trisurf(faceData, transformedVertices(:, 1), transformedVertices(:, 2), transformedVertices(:, 3), 'FaceColor', 'yellow');
        end

        function initializeArms(self)
            self.PrintArm = self.initializePrintArm();
            self.Extruder = self.initializeExtruder();
            self.AssemblyArm = self.initializeAssemblyArm();
        end

        function printArmModel = initializePrintArm(self)
            L1 = Link('d', 0.4, 'a', 0, 'alpha', -pi/2);
            L2 = Link('d', 0, 'a', 0.25, 'alpha', 0);
            L3 = Link('d', 0, 'a', 0.25, 'alpha', 0);
            printArmModel = SerialLink([L1, L2, L3], 'name', 'PrintArm');
            printArmModel.base = transl(0, 0, 0);
            self.plotArm(printArmModel);
        end

        function extruder = initializeExtruder(self)
            extruder.length = 0.1;
            extruder.diameter = 0.02;
        end

        function assemblyArmModel = initializeAssemblyArm(self)
            L1 = Prismatic('theta', 0, 'a', 0, 'alpha', 0, 'qlim', [0, 1]);
            L2 = Link('d', 0.1, 'a', 0, 'alpha', -pi/2);
            L3 = Link('d', 0, 'a', 0.2, 'alpha', 0);
            assemblyArmModel = SerialLink([L1, L2, L3], 'name', 'AssemblyArm');
            assemblyArmModel.base = transl(1, 0, 0);
            self.plotArm(assemblyArmModel);
        end

        function plotArm(self, armModel)
            armModel.plot(armModel.qz, 'workspace', [-2, 2, -2, 2, 0, 2]);
        end

        function executeConstruction(self)
            disp('Starting habitat construction...');
            self.perform3DPrinting();
            self.performAssembly();
            self.checkCompletion();
        end

        function perform3DPrinting(self)
            if ~self.WildlifeDetected
                disp('Performing 3D printing...');
                gcodePath = self.generateGCodePath();
                self.followGCode(self.PrintArm, gcodePath);
            else
                disp('Wildlife detected! Pausing printing.');
            end
        end

        function gcodePath = generateGCodePath(self)
            gcodePath = [linspace(0, 0.5, 50)', linspace(0, 0.5, 50)', linspace(0.4, 0.5, 50)'];
        end

        function followGCode(self, armModel, gcodePath)
            for i = 1:size(gcodePath, 1)
                targetPose = transl(gcodePath(i, 1), gcodePath(i, 2), gcodePath(i, 3));
                q = armModel.ikcon(targetPose);
                armModel.animate(q);
                drawnow;
                self.checkCollision();
                if self.WildlifeDetected
                    disp('Wildlife detected! Stopping movement.');
                    break;
                end
            end
        end

        function performAssembly(self)
            if ~self.WildlifeDetected
                disp('Performing assembly...');
                assemblyPositions = self.getAssemblyPositions();
                for i = 1:length(assemblyPositions)
                    self.moveArmToPosition(self.AssemblyArm, assemblyPositions{i});
                end
            else
                disp('Wildlife detected! Pausing assembly.');
            end
        end

        function positions = getAssemblyPositions(self)
            positions = {
                transl(0.5, 0.5, 0.5);
                transl(0.6, 0.5, 0.5);
            };
        end

        function moveArmToPosition(self, armModel, targetPose)
            q = armModel.ikcon(targetPose);
            qTraj = jtraj(armModel.getpos(), q, 50);
            for j = 1:size(qTraj, 1)
                armModel.animate(qTraj(j, :));
                drawnow;
                self.checkCollision();
                if self.WildlifeDetected
                    disp('Wildlife detected! Stopping movement.');
                    break;
                end
            end
        end

        function checkCollision(self)
            self.WildlifeDetected = false;
        end

        function checkCompletion(self)
            self.TaskComplete = self.performImageProcessing();
            if self.TaskComplete
                disp('Habitat construction and assembly complete.');
            else
                disp('Construction incomplete. Please check the system.');
            end
        end

        function status = performImageProcessing(self)
            status = true;
        end
    end
end
