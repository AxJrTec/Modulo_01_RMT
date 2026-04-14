clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = 13.9;             % Tiempo de simulacion en segundos (s)
ts = 0.1;           % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;      % Vector de tiempo
N = length(t);      % Muestras

%%%%%%%%%%%%%%%%%%%% PARÁMETROS DEL PUZZLEBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R = 0.05;           % Radio de la llanta en metros (5 cm)
L = 0.17;           % Distancia entre ejes en metros (17 cm)

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = zeros (1, N+1);  % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1, N+1);  % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
phi = zeros(1, N+1); % Orientacion del robot en radianes (rad)

x1(1) = 0;   % Posicion inicial eje x
y1(1) = 0;   % Posicion inicial eje y
phi(1) = 0;  % Orientacion inicial del robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%

hx = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)

hx(1) = x1(1); % Posicion en el punto de control del robot en el eje x
hy(1) = y1(1); % Posicion en el punto de control del robot en el eje y

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%

%Tramo 1 (v=1 m/s, w=0 rad/s) por 2s
%Tramo 2 (v=0 m/s, w=-pi/2 rad/s) por 1s
%Tramo 3 (v=1 m/s, w=0 rad/s) por 1.5s
%Tramo 4 (v=0 m/s, w=-pi/2 rad/s) por 1s
%Tramo 5 (v=1 m/s, w=0 rad/s) por 2s
%Tramo 6 (v=0 m/s, w=pi/2 rad/s) por 1s
%Tramo 7 (v=1 m/s, w=0 rad/s) por 2s
%Tramo 8 (v=0 m/s, w=pi/2 rad/s) por 1s
%Tramo 9 (v=1 m/s, w=0 rad/s) por 2.5s

u = [ 1*ones(1,20), 0*ones(1,10), ...
      1*ones(1,15), 0*ones(1,10), ...
      1*ones(1,20), 0*ones(1,10), ...
      1*ones(1,20), 0*ones(1,10), ...
      1*ones(1,25) ];

w = [ 0*ones(1,20), -pi/2*ones(1,10), ...
      0*ones(1,15), -pi/2*ones(1,10), ...
      0*ones(1,20),  pi/2*ones(1,10), ...
      0*ones(1,20),  pi/2*ones(1,10), ...
      0*ones(1,25) ];

%%%%%%%%%%%%%%%%%%%% VELOCIDAD DE RUEDAS %%%%%%%%%%%%%%%%%%%%%%%

% Usando el modelo cinemático inverso del robot diferencial
w_R = (2*u + w*L) / (2*R);  % Velocidad angular rueda derecha (rad/s) 
w_L = (2*u - w*L) / (2*R);  % Velocidad angular rueda izquierda (rad/s)

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N 
    
    phi(k+1) = phi(k) + w(k)*ts; % Integral numérica (método de Euler)
    
    %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%

    xp1 = u(k)*cos(phi(k+1)); 
    yp1 = u(k)*sin(phi(k+1));
    
    phip = w(k);

    x1(k+1) = x1(k) + xp1*ts ; % Integral numérica (método de Euler)
    y1(k+1) = y1(k) + yp1*ts ; % Integral numérica (método de Euler)
    
    % Posicion del robot con respecto al punto de control
    hx(k+1) = x1(k+1); 
    hy(k+1) = y1(k+1);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)', 'Color', [0 0 0], 'FontWeight', 'bold');
ylabel('y(m)', 'Color', [0 0 0], 'FontWeight', 'bold');
zlabel('z(m)', 'Color', [0 0 0], 'FontWeight', 'bold');

view([25 25]); % Orientacion de la figura
axis([-2 6 -6 2 0 2]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2 = plot3(hx(1),hy(1),0,'r','lineWidth',2);

% d) Bucle de simulacion de movimiento del robot

step=1; % pasos para simulacion

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRAFICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph = figure;
set(graph,'position',sizeScreen);

% ===== VELOCIDADES =====
subplot(7,1,1)
plot(t,u,'b','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('u [m/s]')
title('Velocidad lineal')

subplot(7,1,2)
plot(t,w,'r','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('w [rad/s]')
title('Velocidad angular')

% ===== POSICIONES =====
subplot(7,1,3)
plot(t,x1(1:N),'c','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('x [m]')
title('Posición en x')

subplot(7,1,4)
plot(t,y1(1:N),'g','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('y [m]')
title('Posición en y')

subplot(7,1,5)
plot(t,phi(1:N),'m','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('\phi [rad]')
title('Orientación')

% ===== VELOCIDADES DE RUEDAS =====
subplot(7,1,6)
stairs(t,w_R,'b','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('\omega_R [rad/s]')
title('Velocidad rueda derecha')

subplot(7,1,7)
stairs(t,w_L,'r','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('\omega_L [rad/s]')
title('Velocidad rueda izquierda')