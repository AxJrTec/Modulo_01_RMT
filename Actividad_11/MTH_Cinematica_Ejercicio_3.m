clear all
close all
clc

th1 = 3*pi/8; th2 = 5*pi/8;

% Marco base: Z apunta hacia abajo => rotx(180°)
H0 = SE3;

% A1: Sin rotación, tras(0,0,0)
A1 = SE3(rotx(pi), [0 0 0]);

% A2: rotx(-45°) * roty(90°), tras(0,0,0)
A2 = SE3(rotx(-pi/4) * roty(pi/2), [0 0 0]);

% A3: rotx(90°) * roty(-135°), tras(0,0,0)
A3 = SE3(rotx(pi/2) * roty(-3*pi/4), [0 0 0]);

% A4: Sin rotación, tras(7.5*sin(th), 7.5*cos(th), 0)
A4 = SE3(eye(3), [7.5*sin(th1) 7.5*cos(th1) 0]);

% A5: rotz(90°), tras(7.5*sin(th), 7.5*cos(th), 0)
A5 = SE3(rotz(pi/2), [7.5*sin(th2) 7.5*cos(th2) 0]);

% A6: roty(90°) * rotz(90°), tras(0,0,0)
A6 = SE3(roty(pi/2) * rotz(pi/2), [0 0 0]);

% A7: Sin rotación, tras(0, 0, 3)
A7 = SE3(eye(3), [0 0 3]);

% Encadenamiento global
H0_1 = H0  * A1;
H1_2 = H0_1 * A2;
H2_3 = H1_2 * A3;
H3_4 = H2_3 * A4;
H4_5 = H3_4 * A5;
H5_6 = H4_5 * A6;
H6_7 = H5_6 * A7;

% Extraer posiciones de cada origen
frames = {H0, H0_1, H1_2, H2_3, H3_4, H4_5, H5_6, H6_7};
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

% Trama base (H0 con Z hacia abajo) 
trplot(H0, 'rgb', 'axis', ax_lim, 'frame', '0', 'length', 0.8)

% --- Animaciones paso a paso ---
pause;
tranimate(H0,   H0_1, 'rgb', 'axis', ax_lim); pause;
tranimate(H0_1, H1_2, 'rgb', 'axis', ax_lim); pause;
tranimate(H1_2, H2_3, 'rgb', 'axis', ax_lim); pause;
tranimate(H2_3, H3_4, 'rgb', 'axis', ax_lim); pause;
tranimate(H3_4, H4_5, 'rgb', 'axis', ax_lim); pause;
tranimate(H4_5, H5_6, 'rgb', 'axis', ax_lim); pause;
tranimate(H5_6, H6_7, 'rgb', 'axis', ax_lim);