%Limpieza de pantalla
clear all
close all
clc

tic
%Declaración de variables simbólicas
syms x(t) y(t) th(t)  t  %Grados de Libertad del robot móvil

%Creamos el vector de posición 
xi_inercial= [x; y; th];
disp('Coordenadas generalizadas');
pretty (xi_inercial);

%Creamos el vector de velocidades
xip_inercial= diff(xi_inercial, t);
disp('Velocidades generalizadas');
pretty (xip_inercial);
 
%Defino mi vector de posición y matriz de rotación
P(:,:,1)= [x;y;th]; %Viene siendo "xi_inercial"
%Matriz de rotación alrededor del eje z.... 
R(:,:,1)= [cos(th) -sin(th)  0;
           sin(th)  cos(th)  0;
           0        0        1];

%Realizo mi transformación del marco de referencia global al local.... 
xi_local = R(:,:,1)*P(:,:,1)

% Mapeo del sistema global al sistema local de referencia del robot móvil y viceversa.
coordenadas = [
    -5,  9,   -2;   % a
    -3,  8,   63;   % b
     5, -2,   90;   % c
     0,  0,  180;   % d
    -6,  3,  -55;   % e
    10, -2,   45;   % f
     9,  1,   88;   % g
     5,  2,   33;   % h
    -1, -1,   21;   % i
     6,  4,  -40;   % j
     5,  7,   72;   % k
     7,  7,   30;   % l
    11, -4,  360;   % m
    20,  5,  270;   % n
    10,  9,  345;   % ñ
    -9, -8,    8;   % o
     1,  1,   60;   % p
     3,  1,  -30;   % q
    15,  2,  199;   % r
   -10,  0,  300    % s
];

letras = {'a','b','c','d','e','f','g','h','i','j','k','l','m','n','ñ','o','p','q','r','s'};

%Encabezado
fprintf('\n%-5s | %-17s | %-16s | %-8s | %-25s\n', 'Caso', 'Coordenada I.', 'Local (xi_L)', 'Magnitud', 'Vector I. Recuperado');
fprintf('--------------------------------------------------------------------------------------------------------\n');

for i = 1:length(letras)
    %Coordenadas inerciales
    x_val  = coordenadas(i, 1);  % Posicion inicial eje x
    y_val  = coordenadas(i, 2);  % Posicion inicial eje y
    th_deg = coordenadas(i, 3);  % Orientacion inicial del robot
    
    th_rad = deg2rad(th_deg);
     
    %Vector de posición y matriz de rotación
    Pos = [x_val; y_val; 0];
    Rot = [cos(th_rad) -sin(th_rad) 0;
           sin(th_rad)  cos(th_rad) 0;
           0            0           1];
       
    %Transformación del marco de referencia inercial al local
    xi_local = Rot * Pos;
    
    %Magnitud del vector resultante 
    magnitud = sqrt(xi_local(1)^2 + xi_local(2)^2);

    %Mapeo Inverso
    Rot_inv = Rot';
    xi_recuperado = Rot_inv * xi_local;

    %Mostrar resultados
fprintf('%-5s | (%3d, %3d, %4d°) | [%6.2f, %6.2f] | %8.2f | [%6.2f, %6.2f]\n', ...
        letras{i}, x_val, y_val, th_deg, xi_local(1), xi_local(2), magnitud, xi_recuperado(1), xi_recuperado(2));end

toc