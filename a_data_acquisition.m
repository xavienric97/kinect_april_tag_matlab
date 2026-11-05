% =========================================================================
% Script 01 (Vista en Vivo + Captura por Botón de Mouse)
% Description: Muestra video en vivo fluido. Haz clic en el botón de la 
%              pantalla para capturar, y presiona 'Q' o cierra la ventana para salir.
% =========================================================================
clear; clc; close all;

% 1. Limpiar objetos de video previos
delete(imaqfind);

% 2. Cargar parámetros de calibración
calibPath = fullfile('data', 'cameraParams.mat');
if exist(calibPath, 'file')
    load(calibPath, 'cameraParams');
    disp('¡Parámetros de calibración cargados exitosamente!');
else
    error('No se encontró el archivo cameraParams.mat en la carpeta data.');
end

% --- Configuración ---
maxFrames = 15; % Límite de fotogramas deseados

fprintf('Inicializando sensores Kinect v2...\n');
try
    colorCam = videoinput('kinect', 1);
    depthCam = videoinput('kinect', 2);
catch
    error('Kinect v2 no detectado o paquete de soporte no instalado.');
end

% Configurar búferes para streaming continuo pero controlado
set([colorCam, depthCam], 'FramesPerTrigger', Inf);
triggerconfig(colorCam, 'immediate');
triggerconfig(depthCam, 'immediate');

start(colorCam);
start(depthCam);

% Preasignar celdas
rectifiedImages = cell(1, maxFrames);
depthMaps = cell(1, maxFrames);
pointClouds = cell(1, maxFrames);

% Crear la ventana de visualización en vivo optimizada
fig = figure('Name', 'Kinect v2 - Captura por Botón', 'WindowState', 'maximized');
hImage = [];

% --- CREAR EL BOTÓN DE CAPTURA EN LA INTERFAZ ---
% Se ubica en la parte inferior central de la ventana
btnCapture = uicontrol('Style', 'pushbutton', ...
                       'String', 'CAPTURAR FOTO [CLICK]', ...
                       'Units', 'normalized', ...
                       'Position', [0.35, 0.02, 0.3, 0.07], ...
                       'FontSize', 14, ...
                       'FontWeight', 'bold', ...
                       'BackgroundColor', [0 0.7 0.2], ...
                       'ForegroundColor', [1 1 1], ...
                       'Callback', @(~,~) captureFrameCallback());

fprintf('\n=== VISTA EN VIVO CON BOTÓN ACTIVADA ===\n');
fprintf('-> Enfoca tus AprilTags y haz CLIC en el botón verde de la pantalla para capturar.\n');
fprintf('-> Presiona la tecla [Q] o cierra la ventana para terminar y guardar el archivo .mat\n\n');

capturedCount = 0;
running = true;
lastColorImage = [];
lastDepthMap = [];

% Variable de comunicación compartida para el clic del botón
global triggerPressed;
triggerPressed = false;

while running && capturedCount < maxFrames && ishandle(fig)
    
    % Si hay datos nuevos en la cámara, los leemos
    if colorCam.FramesAvailable > 0
        lastColorImage = getsnapshot(colorCam);
        if depthCam.FramesAvailable > 0
            lastDepthMap = getsnapshot(depthCam);
        end
        
        % ¡CLAVE ANTI-RAM!: Vaciar el búfer acumulado
        flushdata(colorCam);
        flushdata(depthCam);
        
        % Rectificar para la visualización en vivo
        rectifiedImage = undistortImage(lastColorImage, cameraParams);
        
        % Mostrar imagen en tiempo real
        if isempty(hImage) || ~ishandle(hImage)
            hImage = imshow(rectifiedImage);
        else
            set(hImage, 'CData', rectifiedImage);
        end
        
        title(sprintf('EN VIVO | Capturados: %d / %d | Haz clic en el botón inferior para capturar', ...
              capturedCount, maxFrames), 'Color', 'b', 'FontSize', 14);
    end
    
    drawnow limitrate;
    
    % --- VERIFICAR SI HICIERON CLIC EN EL BOTÓN ---
    if triggerPressed
        triggerPressed = false; % Resetear indicador
        
        if ~isempty(lastColorImage) && ~isempty(lastDepthMap)
            capturedCount = capturedCount + 1;
            
            % Guardar datos y generar nube de puntos
            rectifiedImages{capturedCount} = undistortImage(lastColorImage, cameraParams);
            depthMaps{capturedCount} = lastDepthMap;
            pointClouds{capturedCount} = pcfromkinect(depthCam, lastDepthMap, lastColorImage);
            
            fprintf('>>> ¡[FOTO %d / %d] Capturada mediante clic y nube de puntos generada!\n', capturedCount, maxFrames);
            
            % Feedback visual temporal en el título
            title(sprintf('¡FOTO %d CAPTURADA!', capturedCount), 'Color', 'g', 'FontSize', 16);
            drawnow;
            pause(0.5);
        else
            fprintf('Advertencia: Aún no hay datos de video listos para capturar.\n');
        end
    end
    
    % Revisar opcionalmente si presionaron 'Q' para salir
    k = get(fig, 'CurrentCharacter');
    if ~isempty(k) && k ~= char(1)
        set(fig, 'CurrentCharacter', char(1));
        if lower(k) == 'q'
            fprintf('Finalizando captura por decisión del usuario.\n');
            running = false;
        end
    end
end

% Limpieza segura del hardware
if ishandle(fig), close(fig); end
stop(colorCam);
stop(depthCam);
delete(colorCam);
delete(depthCam);

% Recortar celdas si se salió antes del límite
if capturedCount < maxFrames
    rectifiedImages = rectifiedImages(1:capturedCount);
    depthMaps = depthMaps(1:capturedCount);
    pointClouds = pointClouds(1:capturedCount);
end

% Guardar archivo .mat final
dataDir = fullfile(pwd, 'data');
if ~exist(dataDir, 'dir'), mkdir(dataDir); end
outputFileName = fullfile(dataDir, 'kinect_sequence.mat');
save(outputFileName, 'rectifiedImages', 'depthMaps', 'pointClouds', '-v7.3');

fprintf('\n=== SESIÓN DE CAPTURA FINALIZADA EXITOSAMENTE ===\n');
fprintf('Se guardaron %d fotogramas limpios en: %s\n', length(rectifiedImages), outputFileName);


% --- FUNCIÓN DE CALLBACK DEL BOTÓN ---
function captureFrameCallback()
    global triggerPressed;
    triggerPressed = true; % Activa la bandera para que el bucle principal procese la foto
end