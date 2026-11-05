% =========================================================================
% Script 02: Estimate AprilTag Poses (Overlapping Tag Network / Multi-Tag Bridge)
% Description: Uses multiple tags per frame to create a connected global map, 
%              allowing the camera to move freely without losing the origin 
%              even if the first tag disappears.
% =========================================================================
clear; clc; close all;

% --- Configuration Parameters ---
dataInputPath = fullfile('data', 'kinect_sequence.mat');
calibPath = fullfile('data', 'cameraParams.mat');
dataOutputPath = fullfile('data', 'camera_poses.mat');

tagFamily = 'tag36h11'; 
tagSize = 0.173; 

if ~exist(dataInputPath, 'file') || ~exist(calibPath, 'file')
    error('Required files missing. Make sure you ran Script 01 and saved cameraParams.mat in data/.');
end

fprintf('Loading image sequence and calibration parameters...\n');
load(dataInputPath, 'rectifiedImages'); 
load(calibPath, 'cameraParams');       

numFrames = length(rectifiedImages);
cameraPoses = cell(1, numFrames);
validFramesCount = 0;

% Dictionary to store the 4x4 global matrix for each detected Tag ID
knownTagMatrices = containers.Map('KeyType', 'double', 'ValueType', 'any');
globalAnchorID = [];

fprintf('=== STARTING MULTI-TAG NETWORK POSE ESTIMATION ===\n');

for i = 1:numFrames
    grayImg = rgb2gray(rectifiedImages{i});
    grayImg = fliplr(grayImg);

    [id, ~, pose] = readAprilTag(grayImg, tagFamily, cameraParams, tagSize);

    if ~isempty(id)
        matCamToGlobal = [];

        % If it is the first frame with tags, define the first seen tag as the Absolute Global Origin
        if isempty(globalAnchorID)
            globalAnchorID = id(1);
            knownTagMatrices(globalAnchorID) = eye(4); % 4x4 Identity matrix (Origin 0,0,0)
            fprintf('>>> [GLOBAL ORIGIN] Frame %d: Tag ID %d set as initial anchor.\n', i, globalAnchorID);
        end

        % Check if any of the visible tags in this frame is already known in our global map
        knownTagInFrame = [];
        for j = 1:length(id)
            if isKey(knownTagMatrices, id(j))
                knownTagInFrame = j;
                break;
            end
        end

        if ~isempty(knownTagInFrame)
            % Bridge Effect! Use the known tag to compute the camera's global position
            tagID = id(knownTagInFrame);
            T_cam_tag = pose(knownTagInFrame);
            mat_cam_tag = T_cam_tag.A;               % Local 4x4 matrix
            mat_global_tag = knownTagMatrices(tagID); % Known global 4x4 matrix
            
            % Compute the global camera matrix using pure matrix algebra
            matCamToGlobal = mat_global_tag * inv(mat_cam_tag);
            
            fprintf('Frame %d/%d: Successful bridge using known Tag ID %d.\n', i, numFrames, tagID);
        else
            fprintf('Frame %d/%d: Warning - No known tags in this frame yet.\n', i, numFrames);
        end

        % If we successfully located the camera, save its pose and register new tags
        if ~isempty(matCamToGlobal)
            cameraPoses{i} = rigidtform3d(matCamToGlobal);
            validFramesCount = validFramesCount + 1;

            % Learn and save the global position of other visible tags in this frame
            for j = 1:length(id)
                tagID = id(j);
                mat_cam_tag = pose(j).A;             
                mat_global_tag = matCamToGlobal * mat_cam_tag;
                
                if ~isKey(knownTagMatrices, tagID)
                    knownTagMatrices(tagID) = mat_global_tag;
                    fprintf('    -> New Tag ID %d connected and registered in the global map.\n', tagID);
                end
            end
        else
            cameraPoses{i} = [];
        end
    else
        cameraPoses{i} = [];
        fprintf('Frame %d/%d: No AprilTags detected.\n', i, numFrames);
    end
end

% Save results
save(dataOutputPath, 'cameraPoses', 'tagSize', 'globalAnchorID', '-v7.3');

fprintf('\n=== MULTI-TAG PROCESS COMPLETED ===\n');
fprintf('Valid global poses calculated for %d out of %d frames.\n', validFramesCount, numFrames);
fprintf('Camera poses saved to: %s\n', dataOutputPath);