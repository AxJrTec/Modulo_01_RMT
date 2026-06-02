clear; clc; close all;

figure('Name', 'Parte 1: 10 Trayectorias', 'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.1 0.9 0.8]);

% a) t en [-2, 2]
t = linspace(-2, 2, 1000);
x = 2*t; y = (t - 3.*t)/3;
subplot(2, 5, 1); plot(x, y, 'b', 'LineWidth', 1.5);
title('a'); xlabel('x'); ylabel('y'); grid on;

% b) t en [0, 10]
t = linspace(0, 10, 1000);
x = t - 3.*sin(t); y = 4 - 3.*cos(t);
subplot(2, 5, 2); plot(x, y, 'r', 'LineWidth', 1.5);
title('b'); xlabel('x'); ylabel('y'); grid on; axis equal;

% c) t en [0, 2*pi]
t = linspace(0, 2*pi, 1000);
x = 3.*cos(t) - cos(3.*t); y = 4.*sin(3.*t);
subplot(2, 5, 3); plot(x, y, 'g', 'LineWidth', 1.5);
title('c'); xlabel('x'); ylabel('y'); grid on; axis equal;

% d) t en [0, 2*pi]
t = linspace(0, 2*pi, 1000);
x = cos(t) + 0.5.*cos(7.*t) + (1/3).*sin(17.*t); y = sin(t) + 0.5.*sin(7.*t) + (1/3).*cos(17.*t);
subplot(2, 5, 4); plot(x, y, 'm', 'LineWidth', 1.5);
title('d'); xlabel('x'); ylabel('y'); grid on; axis equal;

% e) t en [0, 2*pi]
t = linspace(0, 2*pi, 1000);
x = 17.*cos(t) + 7.*cos(17 + 7.*t); y = 17.*sin(t) - 7.*sin(17.*t);
subplot(2, 5, 5); plot(x, y, 'c', 'LineWidth', 1.5);
title('e'); xlabel('x'); ylabel('y'); grid on; axis equal;

% f) t en [0, 14*pi]
t = linspace(0, 14*pi, 1000);
x = 2.*cos(t); y = 2.*sin(t);
subplot(2, 5, 6); plot(x, y, 'k', 'LineWidth', 1.5);
title('f'); xlabel('x'); ylabel('y'); grid on; axis equal;

% g) t en [-2*pi, 2*pi]
t = linspace(-2*pi, 2*pi, 1000);
x = 5.*t - 4.*sin(t); y = 5 - 4.*cos(t);
subplot(2, 5, 7); plot(x, y, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
title('g'); xlabel('x'); ylabel('y'); grid on; axis equal;

% h) t en [0, 2*pi]
t = linspace(0, 2*pi, 1000);
x = 4.*cos(t) + cos(4.*t); y = 4.*sin(t) - sin(4.*t);
subplot(2, 5, 8); plot(x, y, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5);
title('h'); xlabel('x'); ylabel('y'); grid on; axis equal;

% i) t en [0, 2*pi]
t = linspace(0, 2*pi, 1000);
x = sin(2.*t); y = sin(3.*t);
subplot(2, 5, 9); plot(x, y, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);
title('i'); xlabel('x'); ylabel('y'); grid on; axis equal;

% j) t en [0, 2*pi]
t = linspace(0, 2*pi, 1000);
x = sin(4.*t); y = sin(5.*t);
subplot(2, 5, 10); plot(x, y, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
title('j'); xlabel('x'); ylabel('y'); grid on; axis equal;


figure('Name', 'Parte 2: Poligonos Regulares', 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.3 0.8 0.4]);
color_poly = [0.95 0.82 0.47]; 
color_mark_poly = [0.3 0.65 0.85];

n1 = 4; t1 = linspace(0, 2*pi, n1+1);
x1 = cos(t1); y1 = sin(t1);
subplot(1, 3, 1);
plot(x1, y1, 'Color', color_poly, 'LineWidth', 1.2); hold on;
plot(x1(1), y1(1), 'o', 'MarkerEdgeColor', color_mark_poly, 'MarkerSize', 6);
axis equal; axis([-1.1 1.1 -1.1 1.1]); xticks(-1:0.5:1); yticks(-1:0.2:1); box off;

n2 = 6; t2 = linspace(0, 2*pi, n2+1);
x2 = cos(t2); y2 = sin(t2);
subplot(1, 3, 2);
plot(x2, y2, 'Color', color_poly, 'LineWidth', 1.2); hold on;
plot(x2(1), y2(1), 'o', 'MarkerEdgeColor', color_mark_poly, 'MarkerSize', 6);
axis equal; axis([-1.1 1.1 -1.1 1.1]); xticks(-1:0.5:1); yticks(-1:0.2:1); box off;

n3 = 12; t3 = linspace(0, 2*pi, n3+1);
x3 = cos(t3); y3 = sin(t3);
subplot(1, 3, 3);
plot(x3, y3, 'Color', color_poly, 'LineWidth', 1.2); hold on;
plot(x3(1), y3(1), 'o', 'MarkerEdgeColor', color_mark_poly, 'MarkerSize', 6);
axis equal; axis([-1.1 1.1 -1.1 1.1]); xticks(-1:0.5:1); yticks(-1:0.2:1); box off;



figure('Name', 'Parte 3: Curvas de la Rosa', 'Color', 'w', 'Units', 'normalized', 'Position', [0.15 0.2 0.8 0.4]);
color_rosa = [0.89 0.71 0.35]; 
color_mark_rosa = [0.30 0.60 0.85];

t_r1 = linspace(0, 2*pi, 2000); 
r1 = -50 .* cos(4 .* t_r1); 
x_r1 = r1 .* cos(t_r1); y_r1 = r1 .* sin(t_r1);
subplot(1, 3, 1);
plot(x_r1, y_r1, 'Color', color_rosa, 'LineWidth', 1); hold on;
plot(x_r1(1), y_r1(1), 'o', 'MarkerEdgeColor', color_mark_rosa, 'MarkerSize', 6);
axis equal; axis([-55 55 -55 55]); box off;

t_r2 = linspace(0, 4*pi, 2000);
r2 = 5 .* cos(3.5 .* t_r2);
x_r2 = r2 .* cos(t_r2); y_r2 = r2 .* sin(t_r2);
subplot(1, 3, 2);
plot(x_r2, y_r2, 'Color', color_rosa, 'LineWidth', 1); hold on;
plot(x_r2(1), y_r2(1), 'o', 'MarkerEdgeColor', color_mark_rosa, 'MarkerSize', 6);
axis equal; axis([-5.5 5.5 -5.5 5.5]); box off;

t_r3 = linspace(0, 2*pi, 5000); 
r3 = -5 .* cos(40 .* t_r3);
x_r3 = r3 .* cos(t_r3); y_r3 = r3 .* sin(t_r3);
subplot(1, 3, 3);
plot(x_r3, y_r3, 'Color', color_rosa, 'LineWidth', 0.6); hold on;
plot(x_r3(1), y_r3(1), 'o', 'MarkerEdgeColor', color_mark_rosa, 'MarkerSize', 6);
axis equal; axis([-5.5 5.5 -5.5 5.5]); box off;



figure('Name', 'Parte 4: Nuevas Trayectorias', 'Color', 'w', 'Units', 'normalized', 'Position', [0.2 0.1 0.8 0.4]);
color_new = [0.89 0.71 0.35]; 
color_mark_new = [0.30 0.60 0.85];

t_n1 = linspace(0, 2*pi, 1000);
x_n1 = 2 .* cos(t_n1); 
y_n1 = 2 .* sin(t_n1);
subplot(1, 3, 1);
plot(x_n1, y_n1, 'Color', color_new, 'LineWidth', 1); hold on;
plot(x_n1(1), y_n1(1), 'o', 'MarkerEdgeColor', color_mark_new, 'MarkerSize', 6);
axis equal; axis([-2.2 2.2 -2.2 2.2]); box off;

t_n2 = linspace(0, 2*pi, 1000);
x_n2 = cos(t_n2); 
y_n2 = sin(3 .* t_n2);
subplot(1, 3, 2);
plot(x_n2, y_n2, 'Color', color_new, 'LineWidth', 1); hold on;
plot(x_n2(1), y_n2(1), 'o', 'MarkerEdgeColor', color_mark_new, 'MarkerSize', 6);
axis equal; axis([-1.1 1.1 -1.1 1.1]); box off;

t_n3 = linspace(0, 2*pi, 5000);
x_n3 = 1.5 .* cos(t_n3) - cos(17 .* t_n3);
y_n3 = 1.5 .* sin(t_n3) - sin(17 .* t_n3);
subplot(1, 3, 3);
plot(x_n3, y_n3, 'Color', color_new, 'LineWidth', 0.6); hold on;
plot(x_n3(1), y_n3(1), 'o', 'MarkerEdgeColor', color_mark_new, 'MarkerSize', 6);
axis equal; axis([-2.6 2.6 -2.6 2.6]); box off;