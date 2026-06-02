%Limpieza de pantalla
clear all
close all
clc

%Curvas paramétricas
letras = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j'};
paso = 0.01;

for i = 1:length(letras)
    % Definición de t, x, y por casos
    switch letras{i}
        case 'a'
            t = -2:paso:2;
            x = 2*t; 
            y = (t - 3*t)/3;
        case 'b'
            t = 0:paso:10;
            x = t - 3*sin(t); 
            y = 4 - 3*cos(t);
        case 'c'
            t = 0:paso:2*pi;
            x = 3*cos(t) - cos(3*t); 
            y = 4*sin(3*t);
        case 'd'
            t = 0:paso:2*pi;
            x = cos(t) + 1/2*cos(2*t) + 1/3*sin(12*t);
            y = sin(t) + 1/2*sin(2*t) + 1/3*cos(12*t);
        case 'e'
            t = 0:paso:2*pi;
            x = 17*cos(t) + 7*cos(17 + 7*t);
            y = 17*sin(t) - 7*sin(17*t);
        case 'f'
            t = 0:paso:14*pi;
            x = 2*cos(t); 
            y = 2*sin(t);
        case 'g'
            t = -2*pi:paso:2*pi;
            x = 5*t - 4*sin(t); 
            y = 5 - 4*cos(t);
        case 'h'
            t = 0:paso:2*pi;
            x = 4*cos(t) + cos(4*t);
            y = 4*sin(t) - sin(4*t);
        case 'i'
            t = 0:paso:2*pi;
            x = sin(2*t); 
            y = sin(3*t);
        case 'j'
            t = 0:paso:2*pi;
            x = sin(4*t); 
            y = sin(5*t);
    end

    %Graficación en subplots (2 filas, 5 columnas)
    subplot(2, 5, i);
    plot(x, y, 'LineWidth', 1.5);
    title(['Caso ', letras{i}]);
    grid on;
    axis equal;
end
