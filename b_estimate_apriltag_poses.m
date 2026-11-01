% =========================================================================
% Script 02: Estimate AprilTag Poses (Actualizado con Calibración Real)
% Description: Loads rectified RGB frames and precise cameraParams, detects 
%              AprilTags, and estimates camera poses for 3D fusion.
% =========================================================================

clear; clc; close all;

% --- Configuration Parameters ---
dataInputPath = fullfile('data', 'kinect_sequence.mat');
calibPath = fullfile('data', 'cameraParams.mat');
dataOutputPath = fullfile('data', 'camera_poses.mat');

% Specify the AprilTag family used
tagFamily = 'tag36h11'; 

% Specify the physical size of the AprilTag edge in meters (¡Mide tu tag real!)
tagSize = 0.115; 

% Check if required files exist
if ~exist(dataInputPath, 'file') || ~exist(calibPath, 'file')
    error('Faltan archivos necesarios. Asegúrate de haber ejecutado el Script 01 y guardado cameraParams.mat en data/.');
end

fprintf('Cargando secuencia de imágenes y parámetros de calibración...\n');
load(dataInputPath, 'rectifiedImages'); % Usamos las imágenes rectificadas del Script 01
load(calibPath, 'cameraParams');       % Cargamos tu calibración real de 0.14 px

numFrames = length(rectifiedImages);
cameraPoses = cell(1, numFrames);
validFramesCount = 0;

fprintf('=== STARTING APRILTAG DETECTION & POSE ESTIMATION ===\n');

for i = 1:numFrames
    % Convert frame to grayscale for robust tag detection
    grayImg = rgb2gray(rectifiedImages{i});
    
    % Correct horizontal mirror if needed
    grayImg = fliplr(grayImg);
    
    % Detect AprilTags using the precise cameraParams object directly
    [id, loc, pose] = readAprilTag(grayImg, tagFamily, cameraParams, tagSize);
    
    if ~isempty(id)
        % Store the rigid transformation of the first detected tag
        cameraPoses{i} = pose(1); 
        validFramesCount = validFramesCount + 1;
        fprintf('Frame %d/%d: Tag ID %d detected successfully.\n', i, numFrames, id(1));
    else
        cameraPoses{i} = [];
        fprintf('Frame %d/%d: Warning - No AprilTag detected.\n', i, numFrames);
    end
end

% Save computed poses
save(dataOutputPath, 'cameraPoses', 'tagSize', '-v7.3');

fprintf('=== PROCESSING FINISHED ===\n');
fprintf('Poses successfully computed for %d out of %d frames.\n', validFramesCount, numFrames);
fprintf('Camera poses saved to: %s\n', dataOutputPath);