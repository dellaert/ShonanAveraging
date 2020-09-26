function graphWithOutliers = replaceLoopClosureRotationOutliers(graphWithoutOutliers, outliers_percentage)

graphWithOutliers = graphWithoutOutliers;
graphWithOutliers.outliers_id = [];

nrNodes = length(graphWithoutOutliers.pose_estimate);
nrMeasurements = length(graphWithoutOutliers.measurements);
nrLoopClosures = nrMeasurements - nrNodes + 1;

nrOutliersToReplace = floor(nrLoopClosures * outliers_percentage);
fprintf('nrOutliersToReplace=%d\n',nrOutliersToReplace)

for k=1:nrOutliersToReplace % replace the last nrOutliersToReplace measurements
    idOut = nrMeasurements-nrOutliersToReplace+k;
    i = graphWithOutliers.measurements(idOut).i;
    j = graphWithOutliers.measurements(idOut).j;
    if i==j-1 % it's odometry
        error('we are not supposed to replace odometry')
    end
    Rij = eul2rotm([2*pi*rand() 2*pi*rand() 2*pi*rand()]); % outlier!!
   
    graphWithOutliers.measurements(idOut).R = Rij;
    graphWithOutliers.edges(idOut).R = graphWithOutliers.measurements(idOut).R;
end