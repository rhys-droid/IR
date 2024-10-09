 % Changed sphere plot to allow for either point cloud or triangle mesh based on value of a boolean variable, 'makePointCloud'.
            makePointCloud = true;
			makeTriangleMesh = false;

            %% Plot plyFile

            birdOnBranchPos = [0,0,0;0,0,0];
            %birdOnBranch_h = PlaceObject('birdOnBranch.ply', birdOnBranchPos(1,:));
            % 
            % robot = LinearDobot;
            % robot.PlotAndColourRobot;
            % 
            % hold on
            % 
            % pause(0.1);

            
            %% Plot it
            if makePointCloud
                % Plot point cloud

                % points = [X(:),Y(:),Z(:)];
                % spherePc_h = plot3(points(:,1),points(:,2),points(:,3),'r.');

               %points = [X(:),Y(:),Z(:)];
               ptCloud = pcread('birdOnBranch.ply');
               pcshow(ptCloud);
               %spherePc_h = plot3(ptCloud(:,1),ptCloud(:,2),ptCloud(:,3),'r.');



            end
			if makeTriangleMesh
                % Triangle mesh plot
                tri = delaunay(X,Y,Z);
                sphereTri_h = trimesh(tri,X,Y,Z);
            end
                        
            drawnow();
            view(3)
            axis equal