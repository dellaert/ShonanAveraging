clear all
close all
clc

addpath(genpath('../../../outlierRobustEstimation'));

N = 5; % this is the side of the grid. hence the nodes are N^3
prob_loop_closure = 0.4;
outliers_percentage = 0.1;

%% Problem generation
graphWithoutOutliers = generateRandomPosegraph('grid-3d', N, ...
    'LoopClosureProbability', prob_loop_closure, 'RotationStd', 0.01);
problemWithoutOutliers = posegraph2Problem(graphWithoutOutliers, 'OdometryAsPriors', true);
printProblemSummary(problemWithoutOutliers);
plotPosegraph3D(problemWithoutOutliers.graph);
writeGraph(problemWithoutOutliers.graph,[],'inputGrid3D-noOutliers.g2o');

%% Solve it without outliers
system(horzcat('/Users/lucacarlone/Desktop/code/gtsam/build/examples/ShonanAveragingCLI -d 3 ', ...
        '-i /Users/lucacarlone/Desktop/code/ShonanAveraging/gtsam/matlab/inputGrid3D-noOutliers.g2o ', ...
        '-o /Users/lucacarlone/Desktop/code/ShonanAveraging/gtsam/matlab/outputGrid3D-noOutliers.g2o'));
solutionGraphWithoutOutliers = graphDataset3D('/Users/lucacarlone/Desktop/code/ShonanAveraging/gtsam/matlab/outputGrid3D-noOutliers.g2o');
plotPosegraph3D(solutionGraphWithoutOutliers);

%% Add outliers
graphWithOutliers = addLoopClosureRotationOutliers(graphWithoutOutliers, outliers_percentage);
problemWithOutliers = posegraph2Problem(graphWithOutliers, 'OdometryAsPriors', true);
printProblemSummary(problemWithOutliers);
writeGraph(problemWithOutliers.graph,[],'inputGrid3D-withOutliers.g2o');

%% Solve it WITH outliers using Shonan Averaging
disp('========== Solve it WITH outliers using Shonan Averaging ============ ')
system(horzcat('/Users/lucacarlone/Desktop/code/gtsam/build/examples/ShonanAveragingCLI -d 3 ', ...
        '-i /Users/lucacarlone/Desktop/code/ShonanAveraging/gtsam/matlab/inputGrid3D-withOutliers.g2o ', ...
        '-o /Users/lucacarlone/Desktop/code/ShonanAveraging/gtsam/matlab/outputGrid3D-withOutliers.g2o'));
solutionGraphWithOutliers_sa = graphDataset3D('/Users/lucacarlone/Desktop/code/ShonanAveraging/gtsam/matlab/outputGrid3D-withOutliers.g2o');
plotPosegraph3D(solutionGraphWithOutliers_sa);

%% Solve it WITH outliers using Robust Shonan Averaging (using rank pMin = 5)
disp('========== Solve it WITH outliers using ROBUST Shonan Averaging ============ ')
system(horzcat('/Users/lucacarlone/Desktop/code/gtsam/build/examples/ShonanAveragingCLI -d 3 -h true -p 5 ', ...
        '-i /Users/lucacarlone/Desktop/code/ShonanAveraging/gtsam/matlab/inputGrid3D-withOutliers.g2o ', ...
        '-o /Users/lucacarlone/Desktop/code/ShonanAveraging/gtsam/matlab/outputGrid3D-withOutliers.g2o'));
solutionGraphWithOutliers_rsa = graphDataset3D('/Users/lucacarlone/Desktop/code/ShonanAveraging/gtsam/matlab/outputGrid3D-withOutliers.g2o');
plotPosegraph3D(solutionGraphWithOutliers_rsa);

%% Solve with outliers