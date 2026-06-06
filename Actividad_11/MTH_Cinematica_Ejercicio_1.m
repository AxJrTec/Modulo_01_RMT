clear all
close all
clc

% Angulos Ejemplo
th1 = 0; th2 = 0; th3 = 0;

% Matrices de transformación local
H0 = SE3;
H1 = SE3(rotx(pi/2), [0 0 3]);
H2 = SE3(eye(3), [3*cos(th1) 3*sin(th1) 0]);
H3 = SE3(eye(3), [3*cos(th2) 3*sin(th2) 0]);
H4 = SE3(eye(3), [3*cos(th3) 3*sin(th3) 0]);

% Encadenamiento global
H0_1 = H0  * H1;
H1_2 = H0_1 * H2;
H2_3 = H1_2 * H3;
H3_4 = H2_3 * H4;

% Extraer posiciones de cada origen
frames = {H0, H0_1, H1_2, H2_3, H3_4,};
n = length(frames);
pts = zeros(n, 3);
for i = 1:n
    T = frames{i}.T;
    pts(i,:) = T(1:3, 4)';
end

% Figura
figure;
plot3(pts(:,1), pts(:,2), pts(:,3), 'LineWidth', 1.5, 'Color', [1 0.85 0]);
hold on;
grid on;
axis equal;

% Ajustar límites automáticamente con margen
margin = 2;
xlim([min(pts(:,1))-margin, max(pts(:,1))+margin]);
ylim([min(pts(:,2))-margin, max(pts(:,2))+margin]);
zlim([min(pts(:,3))-margin, max(pts(:,3))+margin]);

ax_lim = [min(pts(:,1))-margin, max(pts(:,1))+margin, ...
          min(pts(:,2))-margin, max(pts(:,2))+margin, ...
          min(pts(:,3))-margin, max(pts(:,3))+margin];

% Trama base 
trplot(H0, 'rgb', 'axis', ax_lim, 'frame', '0', 'length', 0.8)

% Animaciones paso a paso 
pause;
tranimate(H0,   H0_1, 'rgb', 'axis', ax_lim); pause;
tranimate(H0_1, H1_2, 'rgb', 'axis', ax_lim); pause;
tranimate(H1_2, H2_3, 'rgb', 'axis', ax_lim); pause;
tranimate(H2_3, H3_4, 'rgb', 'axis', ax_lim);