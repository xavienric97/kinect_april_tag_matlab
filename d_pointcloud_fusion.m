% =========================================================================
% Script 04: Point Cloud Fusion and Global Reconstruction (Corregido Directo)
% Description: Aplica directamente la pose global de la cámara para fusionar 
%              las nubes de puntos en el sistema del mundo sin inversiones.
% =========================================================================

clear; clc; close all;

dataInputPath = fullfile('data', 'kinect_sequence.mat');
posesInputPath = fullfile('data', 'camera_poses.mat');

voxelGridSize = 0.01; % Resolución de 1 cm por vóxel

if ~exist(dataInputPath, 'file') || ~exist(posesInputPath, 'file')
    error('Faltan archivos necesarios. Ejecuta los scripts 01 y 02 primero.');
end

fprintf('Cargando secuencia y poses de la cámara...\n');
load(dataInputPath, 'pointClouds');
load(posesInputPath, 'cameraPoses');

numFrames = length(pointClouds);
globalPointCloud = []; 

fprintf('=== INICIANDO FUSIÓN 3D DIRECTA ===\n');
validMergedCount = 0;

for i = 1:numFrames
    % Verificar que el frame tenga nube de puntos y pose válida calculada
    if i <= length(cameraPoses) && ~isempty(cameraPoses{i}) && ~isempty(pointClouds{i})
        
        ptCloud = pointClouds{i};
        tformCamToGlobal = cameraPoses{i}; % Matriz que ubica la cámara en el mundo global
        
        % 1. Filtrar ruido estorbo del Kinect por cuadro
        ptCloudFiltered = pcdenoise(ptCloud, 'NumNeighbors', 30, 'Threshold', 1.0);
        
        % 2. CORRECCIÓN: Aplicar directamente la matriz de la cámara al mundo 
        % (Sin usar invert(), ya que transporta los puntos locales al marco global).
        ptCloudTransformed = pctransform(ptCloudFiltered, tformCamToGlobal);
        
        % 3. Submuestreo por Voxel Grid para unificar densidad
        ptCloudDownsampled = pcdownsample(ptCloudTransformed, 'gridAverage', voxelGridSize);
        
        % 4. Acumular en el mapa global
        if isempty(globalPointCloud)
            globalPointCloud = ptCloudDownsampled;
        else
            globalPointCloud = pcmerge(globalPointCloud, ptCloudDownsampled, voxelGridSize);
        end
        
        validMergedCount = validMergedCount + 1;
        fprintf('Frame %d fusionado exitosamente.\n', i);
    else
        fprintf('Saltando frame %d (Sin pose o nube de puntos válida).\n', i);
    end
end

if validMergedCount == 0
    error('Ningún frame pudo fusionarse. Revisa el Script 02.');
end

fprintf('=== FUSIÓN COMPLETADA ===\n');
fprintf('Se unificaron %d fotogramas en la reconstrucción global.\n', validMergedCount);

% --- Visualización Final Limpia ---
cleanedCloud = pcdenoise(globalPointCloud, 'NumNeighbors', 40, 'Threshold', 0.8);

figure('Name', 'Reconstrucción Global 3D Optimizada', 'WindowState', 'maximized');
pcshow(cleanedCloud);
view(0, -90); % Vista cenital (superior)
grid on;
xlabel('X (metros)');
ylabel('Y (metros)');
zlabel('Z (metros)');
title(sprintf('Mapa Global Fusión 3D (%d Frames, Voxel: %.3fm)', validMergedCount, voxelGridSize));

% Exportar a archivo PLY para visualización profesional en CloudCompare
outputPlyPath = fullfile('data', 'global_room_reconstruction.ply');
pcwrite(cleanedCloud, outputPlyPath, 'Encoding', 'binary');
fprintf('Nube global exportada exitosamente a: %s\n', outputPlyPath);