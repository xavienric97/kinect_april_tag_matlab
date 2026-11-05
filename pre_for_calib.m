% =========================================================================
% Script de Captura Corregido y Limpio: Calibración RGB (Kinect v2)
% =========================================================================
clear; clc; close all;

% Limpiar cualquier objeto de video previo que pueda bloquear el sensor
delete(imaqfind);

% Crear carpeta de salida si no existe
outputDir = fullfile('data', 'calibration_images');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% --- CONFIGURACIÓN KINECT V2 ---
try
    % Inicializar el Kinect v2 con el adaptador 'kinect' y dispositivo 1 (Color)
    vid = videoinput('kinect', 1);
catch
    error('No se detectó el Kinect v2. Asegúrate de tener instalado el paquete de soporte de Image Acquisition Toolbox.');
end

% Configuración del objeto de video para captura manual
set(vid, 'FramesPerTrigger', 1);
set(vid, 'TriggerRepeat', Inf);
triggerconfig(vid, 'manual');

% Iniciar el flujo de video del sensor
start(vid);

disp('=== INSTRUCCIONES ===');
disp('1. Coloca el tablero de ajedrez frente a la cámara.');
disp('2. Presiona la barra ESPACIADORA para capturar una foto.');
disp('3. Presiona Q para salir cuando tengas entre 15 y 20 fotos.');

imgCount = 0;
fig = figure('Name', 'Captura Calibración Kinect RGB', 'WindowState', 'maximized');
hImage = [];

while ishandle(fig)
    frame = getsnapshot(vid);
    
    if isempty(hImage) || ~ishandle(hImage)
        hImage = imshow(frame);
    else
        set(hImage, 'CData', frame);
    end
    
    title(sprintf('Imágenes capturadas: %d | Presiona ESPACIO para capturar, Q para salir', imgCount));
    drawnow;
    
    if waitforbuttonpress
        key = get(fig, 'CurrentKey');
        if strcmpi(key, 'space')
            imgCount = imgCount + 1;
            filename = fullfile(outputDir, sprintf('calib_img_%02d.png', imgCount));
            imwrite(frame, filename);
            fprintf('¡Imagen %d guardada exitosamente en %s!\n', imgCount, filename);
        elseif strcmpi(key, 'q')
            fprintf('Saliendo del modo de captura...\n');
            break;
        end
    end
end

% --- LIMPIEZA DE RECURSOS ---
stop(vid);
delete(vid);
close all;
fprintf('=== CAPTURA FINALIZADA: Se guardaron %d imágenes en %s ===\n', imgCount, outputDir);

%% Para Calibrar

cameraCalibrator