% =========================================================================
% Script 03: Trajectory Visualization
% Description: Loads estimated camera poses and visualizes the 3D path 
%              traversed by the Kinect sensor during the capture session.
% ========================================================================

clear; clc; close all;

% --- Configuration Parameters ---
posesInputPath = fullfile('data', 'camera_poses.mat'); 

% Check if poses file exists
if ~exist(posesInputPath, 'file')
    error('Input file %s not found. Please run script 02 first.', posesInputPath);
end

fprintf('Loading camera poses from %s...\n', posesInputPath);
load(posesInputPath, 'cameraPoses');

numFrames = length(cameraPoses);

% Extract translation vectors (X, Y, Z coordinates) from valid poses
positions = [];
validFrameIndices = [];

for i = 1:numFrames
    if ~isempty(cameraPoses{i})
        % Extract translation vector [tx, ty, tz] from the rigidtform3d object
        tform = cameraPoses{i};
        trans = tform.Translation; 
        
        positions = [positions; trans];
        validFrameIndices = [validFrameIndices, i];
    end
end

if isempty(positions)
    error('No valid camera poses found in the file. Check your AprilTag detection rate.');
end

fprintf('Successfully extracted trajectory for %d valid frames.\n', size(positions, 1));

% --- Plotting 3D Trajectory ---
figure('Name', 'Kinect 3D Trajectory', 'WindowState', 'maximized');
plot3(positions(:, 1), positions(:, 2), positions(:, 3), '-o', ...
    'LineWidth', 2, ...
    'MarkerSize', 6, ...
    'MarkerFaceColor', 'r', ...
    'Color', [0 0.447 0.741]);

grid on;
hold on;

% Highlight start and end points
plot3(positions(1, 1), positions(1, 2), positions(1, 3), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start (Green)
plot3(positions(end, 1), positions(end, 2), positions(end, 3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % End (Red)

xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');
title('Kinect Camera Trajectory estimated via AprilTag');
legend('Trajectory Path', 'Start Point', 'End Point', 'Location', 'best');
view(3); % Standard 3D view

fprintf('=== TRAJECTORY PLOT GENERATED ===\n');