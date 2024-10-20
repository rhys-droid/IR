classdef cloudPoints < handle

%#ok<*NOPRT>

    methods
        function self = cloudPoints(plyFile, positionxyz)
			        
            if nargin < 1 % Set default values if arguments are not provided
                plyFile = 'birdOnBranch.ply'; 
            end
            if nargin < 2
                positionxyz = [0, 0, 0];  
            end
            
            clf;  


            self.loadPointClouds(plyFile, positionxyz);
        end
    end

    methods (Static)
%% Function to load ply file and its cloud points that has output of the XYZ limits of cloud points

        function cloudPointLocation = loadPointClouds(plyFile, positionxyz)

            if size(positionxyz) ~= 3 % Throwing error if there is invalid input
                error('Invalid position input. Ensure format is [x, y, z]');
            end

            hold on

            ptCloud = pcread(plyFile); % Reading PLY point clouds
            
            PtCloudTrans = pointCloud(ptCloud.Location+positionxyz); % Translating point clouds to desired position
            
            pcshow(PtCloudTrans); % Showing translated point clouds
            
            %xyzLimits = [PtCloudTrans.XLimits; PtCloudTrans.YLimits; PtCloudTrans.ZLimits]; % Storing value of xyz limits of cloud points

            xyzLocation = PtCloudTrans.Location;

            PlaceObject(plyFile, positionxyz);
            
            pause(0.1);
            
            cloudPointLocation = xyzLocation; % Output is xyz limits of ply file
          
        end
    end 
end

