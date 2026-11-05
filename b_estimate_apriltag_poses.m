% =========================================================================
% Script 02: Estimate AprilTag Poses (Red de Tags con Solapamiento / Multi-Tag Bridge)
% Description: Utiliza múltiples tags por fotograma para crear un mapa global 
%              conectado, permitiendo que la cámara se mueva libremente sin perder 
%              el origen aunque el primer tag desaparezca.
% =========================================================================
clear; clc; close all;

% --- Configuration Parameters ---
dataInputPath = fullfile('data', 'kinect_sequence.mat');
calibPath = fullfile('data', 'cameraParams.mat');
dataOutputPath = fullfile('data', 'camera_poses.mat');

tagFamily = 'tag36h11'; 
tagSize = 0.173; 

if ~exist(dataInputPath, 'file') || ~exist(calibPath, 'file')
    error('Faltan archivos necesarios. Asegúrate de haber ejecutado el Script 01 y guardado cameraParams.mat en data/.');
end

fprintf('Cargando secuencia de imágenes y parámetros de calibración...\n');
load(dataInputPath, 'rectifiedImages'); 
load(calibPath, 'cameraParams');       

numFrames = length(rectifiedImages);
cameraPoses = cell(1, numFrames);
validFramesCount = 0;

% Diccionario para almacenar la matriz 4x4 global de cada Tag ID encontrado
knownTagMatrices = containers.Map('KeyType', 'double', 'ValueType', 'any');
globalAnchorID = [];

fprintf('=== STARTING MULTI-TAG NETWORK POSE ESTIMATION ===\n');

for i = 1:numFrames
    grayImg = rgb2gray(rectifiedImages{i});
    grayImg = fliplr(grayImg);

    [id, ~, pose] = readAprilTag(grayImg, tagFamily, cameraParams, tagSize);

    if ~isempty(id)
        matCamToGlobal = [];

        % Si es el primer frame con tags, definimos el primer tag visto como el Origen Global Absoluto
        if isempty(globalAnchorID)
            globalAnchorID = id(1);
            knownTagMatrices(globalAnchorID) = eye(4); % Matriz identidad 4x4 (Origen 0,0,0)
            fprintf('>>> [ORIGEN GLOBAL] Frame %d: Tag ID %d fijado como el ancla inicial.\n', i, globalAnchorID);
        end

        % Buscar si alguno de los tags visibles en este frame YA es conocido en nuestro mapa global
        knownTagInFrame = [];
        for j = 1:length(id)
            if isKey(knownTagMatrices, id(j))
                knownTagInFrame = j;
                break;
            end
        end

        if ~isempty(knownTagInFrame)
            % ¡Efecto Puente! Usamos el tag conocido para calcular la posición global de la cámara
            tagID = id(knownTagInFrame);
            T_cam_tag = pose(knownTagInFrame);
            mat_cam_tag = T_cam_tag.A;               % CORREGIDO: Usar .A para rigidtform3d
            mat_global_tag = knownTagMatrices(tagID); % Matriz 4x4 global conocida
            
            % Calcular la matriz global de la cámara usando álgebra matricial pura
            matCamToGlobal = mat_global_tag * inv(mat_cam_tag);
            
            fprintf('Frame %d/%d: Puente exitoso usando Tag ID %d conocido.\n', i, numFrames, tagID);
        else
            fprintf('Frame %d/%d: Advertencia - Ningún tag conocido en este frame todavía.\n', i, numFrames);
        end

        % Si logramos ubicar la cámara, guardamos su pose y registramos los tags nuevos
        if ~isempty(matCamToGlobal)
            cameraPoses{i} = rigidtform3d(matCamToGlobal);
            validFramesCount = validFramesCount + 1;

            % Aprender y guardar la posición global de los demás tags visibles en este cuadro
            for j = 1:length(id)
                tagID = id(j);
                mat_cam_tag = pose(j).A;             % CORREGIDO: Usar .A para rigidtform3d
                mat_global_tag = matCamToGlobal * mat_cam_tag;
                
                if ~isKey(knownTagMatrices, tagID)
                    knownTagMatrices(tagID) = mat_global_tag;
                    fprintf('    -> Nuevo Tag ID %d conectado y registrado en el mapa global.\n', tagID);
                end
            end
        else
            cameraPoses{i} = [];
        end
    else
        cameraPoses{i} = [];
        fprintf('Frame %d/%d: No se detectaron AprilTags.\n', i, numFrames);
    end
end

% Guardar resultados
save(dataOutputPath, 'cameraPoses', 'tagSize', 'globalAnchorID', '-v7.3');

fprintf('\n=== PROCESO MULTI-TAG TERMINADO ===\n');
fprintf('Poses globales válidas calculadas para %d de %d frames.\n', validFramesCount, numFrames);
fprintf('Camera poses saved to: %s\n', dataOutputPath);