function graphWithOutliers = addLoopClosureRotationOutliers(graphWithoutOutliers, outliers_percentage)

graphWithOutliers = graphWithoutOutliers;
graphWithOutliers.outliers_id = [];

nrNodes = length(graphWithoutOutliers.pose_estimate);
nrMeasurements = length(graphWithoutOutliers.measurements);
nrLoopClosures = nrMeasurements - nrNodes + 1;

nrOutliersToAdd = nrLoopClosures * outliers_percentage / (1-outliers_percentage);

poses_gt = graphWithoutOutliers.poses_gt;

for k=1:nrOutliersToAdd
    i = randi(nrNodes);
    j = randi(nrNodes);
    while i==j && i==j-1 % make sure i and j are different
        j = randi(nrNodes);
    end
    % Rij_gt = poses_gt(i).R' * poses_gt(j).R;
    Rij = eul2rotm([2*pi*rand() 2*pi*rand() 2*pi*rand()]); % outlier!!
    tij_gt = poses_gt(i).R' * ( poses_gt(j).t - poses_gt(i).t );
    tij = tij_gt; % we still use clean translations
    graphWithOutliers.measurements(nrMeasurements+1).i = i;
    graphWithOutliers.measurements(nrMeasurements+1).j = j;
    graphWithOutliers.measurements(nrMeasurements+1).R = Rij;
    graphWithOutliers.measurements(nrMeasurements+1).t = tij;
    graphWithOutliers.measurements(nrMeasurements+1).tau = graphWithOutliers.measurements(nrMeasurements).tau;
    graphWithOutliers.measurements(nrMeasurements+1).kappa = graphWithOutliers.measurements(nrMeasurements).kappa;
    graphWithOutliers.measurements(nrMeasurements+1).Omega = graphWithOutliers.measurements(nrMeasurements).Omega; 
    
    graphWithOutliers.edges(nrMeasurements+1).i = i;
    graphWithOutliers.edges(nrMeasurements+1).j = j;
    graphWithOutliers.edges(nrMeasurements+1).R = graphWithOutliers.measurements(nrMeasurements+1).R;
    graphWithOutliers.edges(nrMeasurements+1).t = graphWithOutliers.measurements(nrMeasurements+1).t;
    graphWithOutliers.edges(nrMeasurements+1).Info = graphWithOutliers.edges(nrMeasurements).Info;
    
    % graphWithOutliers.outliers_id = [graphWithOutliers.outliers_id nrMeasurements+1];
    
    nrMeasurements = nrMeasurements+1;
end