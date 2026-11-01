% =========================================================================
% Script 04: Point Cloud Fusion and Global Reconstruction
% Description: Loads individual point clouds and camera poses, transforms 
%              each frame to the global AprilTag coordinate system, filters 
%              noise, and merges them into a single 3D room map.
% ========================================================================

clear; clc; close all;

% --- Configuration Parameters ---
dataInputPath = fullfile('data', 'kinect_sequence.mat');
posesInputPath = fullfile('data', 'camera_poses.mat');

% Voxel grid filter size for downsampling (e.g., 0.01m = 1 cm resolution)
% Lower = denser/more detailed, Higher = faster/lighter
voxelGridSize = 0.01; 

% Check if required files exist
if ~exist(dataInputPath, 'file') || ~exist(posesInputPath, 'file')
    error('Required data files not found. Please run scripts 01 and 02 first.');
end

fprintf('Loading dataset and camera poses...\n');
load(dataInputPath, 'pointClouds');
load(posesInputPath, 'cameraPoses');

numFrames = length(pointClouds);
globalPointCloud = []; % Start with an empty array/flag

fprintf('=== STARTING 3D POINT CLOUD FUSION ===\n');

validMergedCount = 0;

% =========================================================================
% Fragmento clave corregido para el Script 04
% =========================================================================

% Transformación extrínseca aproximada entre Depth y RGB en Kinect v2 
% (Rotación identidad y pequeña traslación en X/Y según especificaciones del SDK)
% O alternativamente, asegurando el filtrado de ruido por cuadro:

for i = 1:numFrames
    % Check if this frame has a valid point cloud and a computed camera pose
    if i <= length(cameraPoses) && ~isempty(cameraPoses{i}) && ~isempty(pointClouds{i})
        
        ptCloud = pointClouds{i};
        tform = cameraPoses{i};
        
        % 1. CORRECCIÓN CLAVE: Invertir la transformación (de cámara a mundo global del tag)
        tformGlobal = invert(tform);
        
        % 2. Filtrado opcional de ruido por cuadro
        ptCloudFiltered = pcdenoise(ptCloud, 'NumNeighbors', 30, 'Threshold', 1.0);
        
        % 3. Transformar la nube al sistema global usando la matriz invertida[cite: 4]
        ptCloudTransformed = pctransform(ptCloudFiltered, tformGlobal);
        
        % 4. Submuestreo por Voxel Grid para optimizar densidad[cite: 4]
        ptCloudDownsampled = pcdownsample(ptCloudTransformed, 'gridAverage', voxelGridSize);
        
        % 5. Fusionar en el mapa global acumulado[cite: 4]
        if isempty(globalPointCloud)
            globalPointCloud = ptCloudDownsampled;
        else
            globalPointCloud = pcmerge(globalPointCloud, ptCloudDownsampled, voxelGridSize);
        end
        
        validMergedCount = validMergedCount + 1;
        fprintf('Merged frame %d into global map.\n', i);
    else
        fprintf('Skipping frame %d (No valid pose or point cloud).\n', i);
    end
end

if validMergedCount == 0
    error('No frames were successfully merged. Check your AprilTag poses.');
end

fprintf('=== FUSION COMPLETE ===\n');
fprintf('Successfully fused %d frames into the global reconstruction.\n', validMergedCount);

%% --- Visualization of the Final 3D Room ---
figure('Name', 'Global 3D Room Reconstruction', 'WindowState', 'maximized');
pcshow(globalPointCloud);

% Configure view to top-down / YX plane as established in your project conventions
view(0, -90);


xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');
title(sprintf('Global Fused Room Map (%d Frames, Voxel: %.3fm)', validMergedCount, voxelGridSize));

% Optional: Save the final global point cloud as a .ply file for CloudCompare
outputPlyPath = fullfile('data', 'global_room_reconstruction.ply');
pcwrite(globalPointCloud, outputPlyPath, 'Encoding', 'binary');
fprintf('Global point cloud exported to: %s\n', outputPlyPath);

%% gjhgjh

% Después de fusionar todos los cuadros en ptCloudGlobal:
% 'NumNeighbors' define cuántos vecinos analiza, y 'Threshold' la sensibilidad
cleanedCloud = pcdenoise(globalPointCloud, 'NumNeighbors', 30, 'Threshold', 1.0);

figure('Name', 'Nube Limpia', 'WindowState', 'maximized');
pcshow(cleanedCloud);
view(0, -90);
title('Global Point Cloud (Sin Ruido / Denoised)');
grid on;