% =========================================================================
% Script 01 (Live View + Mouse Button Capture)
% Description: Displays smooth live video. Click the on-screen button to 
%              capture, and press 'Q' or close the window to exit.
% =========================================================================
clear; clc; close all;

% 1. Clean up previous video objects
delete(imaqfind);

% 2. Load calibration parameters
calibPath = fullfile('data', 'cameraParams.mat');
if exist(calibPath, 'file')
    load(calibPath, 'cameraParams');
    disp('Calibration parameters loaded successfully!');
else
    error('cameraParams.mat file not found in the data folder.');
end

% --- Configuration ---
maxFrames = 20; % Desired frame limit

fprintf('Initializing Kinect v2 sensors...\n');
try
    colorCam = videoinput('kinect', 1);
    depthCam = videoinput('kinect', 2);
catch
    error('Kinect v2 not detected or support package not installed.');
end

% Configure buffers for continuous yet controlled streaming
set([colorCam, depthCam], 'FramesPerTrigger', Inf);
triggerconfig(colorCam, 'immediate');
triggerconfig(depthCam, 'immediate');

start(colorCam);
start(depthCam);

% Preallocate cells
rectifiedImages = cell(1, maxFrames);
depthMaps = cell(1, maxFrames);
pointClouds = cell(1, maxFrames);

% Create optimized live view window
fig = figure('Name', 'Kinect v2 - Button Capture', 'WindowState', 'maximized');
hImage = [];

% --- CREATE INTERFACE CAPTURE BUTTON ---
% Positioned at the bottom center of the window
btnCapture = uicontrol('Style', 'pushbutton', ...
                       'String', 'CAPTURE PHOTO [CLICK]', ...
                       'Units', 'normalized', ...
                       'Position', [0.35, 0.02, 0.3, 0.07], ...
                       'FontSize', 14, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0 0.7 0.2], ...
                       'ForegroundColor', [1 1 1], ...
                       'Callback', @(~,~) captureFrameCallback());

fprintf('\n=== LIVE VIEW WITH BUTTON ACTIVATED ===\n');
fprintf('-> Focus your AprilTags and CLICK the green button on the screen to capture.\n');
fprintf('-> Press the [Q] key or close the window to finish and save the .mat file\n\n');

capturedCount = 0;
running = true;
lastColorImage = [];
lastDepthMap = [];

% Shared communication variable for button click
global triggerPressed;
triggerPressed = false;

while running && capturedCount < maxFrames && ishandle(fig)
    
    % Read new data from the camera if available
    if colorCam.FramesAvailable > 0
        lastColorImage = getsnapshot(colorCam);
        if depthCam.FramesAvailable > 0
            lastDepthMap = getsnapshot(depthCam);
        end
        
        % ANTI-RAM KEY: Flush the accumulated buffer
        flushdata(colorCam);
        flushdata(depthCam);
        
        % Rectify for live visualization
        rectifiedImage = undistortImage(lastColorImage, cameraParams);
        
        % Display image in real time
        if isempty(hImage) || ~ishandle(hImage)
            hImage = imshow(rectifiedImage);
        else
            set(hImage, 'CData', rectifiedImage);
        end
        
        title(sprintf('LIVE | Captured: %d / %d | Click the bottom button to capture', ...
              capturedCount, maxFrames), 'Color', 'b', 'FontSize', 14);
    end
    
    drawnow limitrate;
    
    % --- CHECK IF BUTTON WAS CLICKED ---
    if triggerPressed
        triggerPressed = false; % Reset flag
        
        if ~isempty(lastColorImage) && ~isempty(lastDepthMap)
            capturedCount = capturedCount + 1;
            
            % Save data and generate point cloud
            rectifiedImages{capturedCount} = undistortImage(lastColorImage, cameraParams);
            depthMaps{capturedCount} = lastDepthMap;
            pointClouds{capturedCount} = pcfromkinect(depthCam, lastDepthMap, lastColorImage);
            
            fprintf('>>> [FRAME %d / %d] Captured via click and point cloud generated successfully!\n', capturedCount, maxFrames);
            
            % Temporary visual feedback in title
            title(sprintf('FRAME %d CAPTURED!', capturedCount), 'Color', 'g', 'FontSize', 16);
            drawnow;
            pause(0.5);
        else
            fprintf('Warning: Video data not ready for capture yet.\n');
        end
    end
    
    % Optionally check if 'Q' was pressed to exit
    k = get(fig, 'CurrentCharacter');
    if ~isempty(k) && k ~= char(1)
        set(fig, 'CurrentCharacter', char(1));
        if lower(k) == 'q'
            fprintf('Finalizing capture by user request.\n');
            running = false;
        end
    end
end

% Safe hardware cleanup
if ishandle(fig), close(fig); end
stop(colorCam);
stop(depthCam);
delete(colorCam);
delete(depthCam);

% Trim cells if exited before the limit
if capturedCount < maxFrames
    rectifiedImages = rectifiedImages(1:capturedCount);
    depthMaps = depthMaps(1:capturedCount);
    pointClouds = pointClouds(1:capturedCount);
end

% Save final .mat file
dataDir = fullfile(pwd, 'data');
if ~exist(dataDir, 'dir'), mkdir(dataDir); end
outputFileName = fullfile(dataDir, 'kinect_sequence.mat');
save(outputFileName, 'rectifiedImages', 'depthMaps', 'pointClouds', '-v7.3');

fprintf('\n=== CAPTURE SESSION COMPLETED SUCCESSFULLY ===\n');
fprintf('Saved %d clean frames to: %s\n', length(rectifiedImages), outputFileName);


% --- BUTTON CALLBACK FUNCTION ---
function captureFrameCallback()
    global triggerPressed;
    triggerPressed = true; % Activates flag for the main loop to process the photo
end