% =========================================================================
% SubScript a2_inspect_data: Inspect Recorded Kinect Frames
% Description: Loads the saved Kinect sequence and visualizes synchronized 
%              RGB images alongside their corresponding 3D point clouds.
% =========================================================================
clear; clc; close all;

% Load the recorded data
dataFileName = fullfile('data', 'kinect_sequence.mat');
if ~exist(dataFileName, 'file')
    error('File %s not found. Please run data acquisition first.', dataFileName);
end

fprintf('Loading data from %s...\n', dataFileName);
load(dataFileName, 'rectifiedImages', 'pointClouds');

numFrames = length(pointClouds);
fprintf('Total frames loaded: %d\n', numFrames);

% Select frames to inspect (First, Middle, and Last frame)
framesToInspect = [1, round(numFrames / 2), numFrames];

for i = 1:length(framesToInspect)
    idx = framesToInspect(i);
    
    % Ensure the index is valid
    if idx < 1 || idx > numFrames, continue; end
    
    % Create a large figure window to view both plots side-by-side
    figure('Name', sprintf('Inspection - Frame %d of %d', idx, numFrames), ...
           'Position', [100, 100, 1200, 500]);
    
    % 1. Left Panel: RGB Image with horizontal mirror correction (fliplr)
    subplot(1, 2, 1);
    imshow(fliplr(rectifiedImages{idx}));
    title(sprintf('RGB Image - Corrected (Frame %d)', idx), 'FontSize', 12);
    
    % 2. Right Panel: Point Cloud (Top-down view / YX plane)
    subplot(1, 2, 2);
    pcshow(pointClouds{idx});
    view(0, -90); % Set top-down perspective view
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(sprintf('Point Cloud - YX Plane (Frame %d)', idx), 'FontSize', 12);
end

fprintf('Inspection complete!\n');