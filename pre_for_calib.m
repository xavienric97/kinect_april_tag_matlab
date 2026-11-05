% =========================================================================
% Corrected and Clean Capture Script: RGB Calibration (Kinect v2)
% =========================================================================
clear; clc; close all;

% Clear any previous video objects that might lock the sensor
delete(imaqfind);

% Create output folder if it doesn't exist
outputDir = fullfile('data', 'calibration_images');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% --- KINECT V2 CONFIGURATION ---
try
    % Initialize Kinect v2 with the 'kinect' adapter and device 1 (Color)
    vid = videoinput('kinect', 1);
catch
    error('Kinect v2 not detected. Make sure you have the Image Acquisition Toolbox Support Package installed.');
end

% Configure video object for manual capture
set(vid, 'FramesPerTrigger', 1);
set(vid, 'TriggerRepeat', Inf);
triggerconfig(vid, 'manual');

% Start the video stream from the sensor
start(vid);

disp('=== INSTRUCTIONS ===');
disp('1. Place the checkerboard pattern in front of the camera.');
disp('2. Press the SPACEBAR to capture a photo.');
disp('3. Press Q to exit when you have between 15 and 20 photos.');

imgCount = 0;
fig = figure('Name', 'Kinect RGB Calibration Capture', 'WindowState', 'maximized');
hImage = [];

while ishandle(fig)
    frame = getsnapshot(vid);
    
    if isempty(hImage) || ~ishandle(hImage)
        hImage = imshow(frame);
    else
        set(hImage, 'CData', frame);
    end
    
    title(sprintf('Captured images: %d | Press SPACE to capture, Q to exit', imgCount));
    drawnow;
    
    if waitforbuttonpress
        key = get(fig, 'CurrentKey');
        if strcmpi(key, 'space')
            imgCount = imgCount + 1;
            filename = fullfile(outputDir, sprintf('calib_img_%02d.png', imgCount));
            imwrite(frame, filename);
            fprintf('Image %d successfully saved to %s!\n', imgCount, filename);
        elseif strcmpi(key, 'q')
            fprintf('Exiting capture mode...\n');
            break;
        end
    end
end

% --- RESOURCE CLEANUP ---
stop(vid);
delete(vid);
close all;

fprintf('=== CAPTURE COMPLETED: Saved %d images in %s ===\n', imgCount, outputDir);

%% For Calibration
cameraCalibrator