% =========================================================================
% Script 04: Point Cloud Fusion and Global Reconstruction (Direct Correction)
% Description: Directly applies the global camera pose to fuse point 
%              clouds into the world coordinate system without inversions.
% =========================================================================

clear; clc; close all;

dataInputPath = fullfile('data', 'kinect_sequence.mat');
posesInputPath = fullfile('data', 'camera_poses.mat');

voxelGridSize = 0.01; % Voxel resolution of 1 cm

if ~exist(dataInputPath, 'file') || ~exist(posesInputPath, 'file')
    error('Required data files not found. Please run scripts 01 and 02 first.');
end

fprintf('Loading dataset sequence and camera poses...\n');
load(dataInputPath, 'pointClouds');
load(posesInputPath, 'cameraPoses');

numFrames = length(pointClouds);
globalPointCloud = []; 

fprintf('=== STARTING DIRECT 3D POINT CLOUD FUSION ===\n');
validMergedCount = 0;

for i = 1:numFrames
    % Check if this frame has a valid point cloud and a computed camera pose
    if i <= length(cameraPoses) && ~isempty(cameraPoses{i}) && ~isempty(pointClouds{i})
        
        ptCloud = pointClouds{i};
        tformCamToGlobal = cameraPoses{i}; % Matrix locating the camera in the global world
        
        % 1. Filter out Kinect noise per frame
        ptCloudFiltered = pcdenoise(ptCloud, 'NumNeighbors', 30, 'Threshold', 1.0);
        
        % 2. CORRECTION: Directly apply the camera-to-world matrix 
        % (Without invert(), since it maps local sensor points to the global frame).
        ptCloudTransformed = pctransform(ptCloudFiltered, tformCamToGlobal);
        
        % 3. Voxel Grid downsampling to unify point density
        ptCloudDownsampled = pcdownsample(ptCloudTransformed, 'gridAverage', voxelGridSize);
        
        % 4. Accumulate into the global map
        if isempty(globalPointCloud)
            globalPointCloud = ptCloudDownsampled;
        else
            globalPointCloud = pcmerge(globalPointCloud, ptCloudDownsampled, voxelGridSize);
        end
        
        validMergedCount = validMergedCount + 1;
        fprintf('Frame %d successfully merged into global map.\n', i);
    else
        fprintf('Skipping frame %d (No valid pose or point cloud).\n', i);
    end
end

if validMergedCount == 0
    error('No frames were successfully merged. Check Script 02.');
end

fprintf('=== FUSION COMPLETE ===\n');
fprintf('Successfully fused %d frames into the global reconstruction.\n', validMergedCount);

% --- Clean Final Visualization ---
cleanedCloud = pcdenoise(globalPointCloud, 'NumNeighbors', 40, 'Threshold', 0.8);

figure('Name', 'Optimized Global 3D Reconstruction', 'WindowState', 'maximized');
pcshow(cleanedCloud);
view(0, -90); % Top-down (overhead) view
grid on;
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');
title(sprintf('Global Fused 3D Map (%d Frames, Voxel: %.3fm)', validMergedCount, voxelGridSize));

% Export global point cloud as a PLY file for professional viewing in CloudCompare
outputPlyPath = fullfile('data', 'global_room_reconstruction.ply');
pcwrite(cleanedCloud, outputPlyPath, 'Encoding', 'binary');
fprintf('Global point cloud successfully exported to: %s\n', outputPlyPath);