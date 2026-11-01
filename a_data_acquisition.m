% =========================================================================
% Script 01: Data Acquisition & Rectification from Kinect v2
% Description: Captures synchronized RGB images and Point Clouds at a 
%              controlled low frame rate (2 FPS), rectifying the RGB frames 
%              using cameraParams.mat.
% =========================================================================

clear; clc; close all;

% 1. Limpiar objetos de video previos para evitar errores de bloqueo
delete(imaqfind);

% 2. Cargar los parámetros de calibración desde la carpeta data
calibPath = fullfile('data', 'cameraParams.mat');
if exist(calibPath, 'file')
    load(calibPath, 'cameraParams');
    disp('¡Parámetros de calibración cargados exitosamente!');
else
    error('No se encontró el archivo cameraParams.mat en la carpeta data. Realiza la calibración primero.');
end

% --- Configuration Parameters ---
targetFPS = 2;                  % Desired frames per second (2 FPS)
intervalSec = 1 / targetFPS;    % Time interval between captures
durationSec = 25;              % Total duration of capture in seconds (ajusta a tu gusto)
maxFrames = targetFPS * durationSec; % Total frames to capture

fprintf('Initializing Kinect v2 sensors...\n');

% Initialize video input objects for Kinect v2
try
    colorCam = videoinput('kinect', 1);
    depthCam = videoinput('kinect', 2);
catch
    error('Kinect v2 not detected or support package not installed properly.');
end

% Configure trigger settings for sequential capture
set([colorCam, depthCam], 'FramesPerTrigger', 1);
set([colorCam, depthCam], 'TriggerRepeat', maxFrames - 1);
triggerconfig(colorCam, 'manual');
triggerconfig(depthCam, 'manual');

start(colorCam);
start(depthCam);

% Preallocate cell arrays for efficiency
colorImages = cell(1, maxFrames);
rectifiedImages = cell(1, maxFrames);
pointClouds = cell(1, maxFrames);

fprintf('Ready! Starting capture in 3 seconds...\n');
pause(3);

fprintf('=== RECORDING STARTED (Target: %d frames at %d FPS) ===\n', maxFrames, targetFPS);

% Main capture loop
for i = 1:maxFrames
    loopStart = tic;
    
    % Trigger and acquire synchronized frames
    trigger(colorCam);
    trigger(depthCam);
    
    colorImage = getdata(colorCam, 1);
    depthMap = getdata(depthCam, 1);
    
    % --- APLICAR RECTIFICACIÓN GEOMÉTRICA CON CAMERA PARAMS ---
    rectifiedImage = undistortImage(colorImage, cameraParams);
    
    % Generate Point Cloud from Kinect depth and color data
    ptCloud = pcfromkinect(depthCam, depthMap, colorImage);
    
    % Store in cell arrays
    colorImages{i} = colorImage;       % Original por si acaso
    rectifiedImages{i} = rectifiedImage; % Versión limpia de distorsión
    pointClouds{i} = ptCloud;
    
    fprintf('Captured and rectified frame %d / %d\n', i, maxFrames);
    
    % Enforce the target frame rate interval (2 FPS)
    elapsedTime = toc(loopStart);
    remainingTime = intervalSec - elapsedTime;
    if remainingTime > 0
        pause(remainingTime);
    end
end

% Clean up hardware objects
stop(colorCam);
stop(depthCam);
delete(colorCam);
delete(depthCam);

% Ensure the 'data' directory exists in the current working folder
dataDir = fullfile(pwd, 'data');
if ~exist(dataDir, 'dir')
    mkdir(dataDir);
end

% Save data to .mat file (-v7.3 supports large variables)
outputFileName = fullfile(dataDir, 'kinect_sequence.mat');
save(outputFileName, 'colorImages', 'rectifiedImages', 'pointClouds', '-v7.3');

fprintf('=== RECORDING FINISHED ===\n');
fprintf('Data successfully saved to: %s\n', outputFileName);