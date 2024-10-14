 % Changed sphere plot to allow for either point cloud or triangle mesh based on value of a boolean variable, 'makePointCloud'.
    makePointCloud = true;
    
    %% Plot plyFile
    
    birdOnBranchPos = [3,2,0;0,0,0];
    birdOnBranch_h = PlaceObject('birdOnBranch.ply', birdOnBranchPos(1,:));
    hold on
    % 
    % robot = LinearDobot;
    % robot.PlotAndColourRobot;
    % 
    % hold on
    % 
    % pause(0.1);
    
    %% Plot it
    if makePointCloud
    
      
       PtCloud = pcread('birdOnBranch.ply'); %Reading PLY point clouds
    
       birdOnBranchPtCloudTrans = pointCloud(ptCloud.Location+birdOnBranchPos(1,:)); % Translating point clouds to desired position
    
       pcshow(birdOnBranchPtCloudTrans); % Showing translated point clouds
    
       birdOnBranchXYZLimits = [birdOnBranchPtCloudTrans.XLimits; birdOnBranchPtCloudTrans.YLimits; birdOnBranchPtCloudTrans.ZLimits];
    
    
    
       hold off
    end
                
    drawnow();
    view(3)
    axis equal