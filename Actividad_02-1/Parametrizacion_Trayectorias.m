%Limpieza de pantalla
clear all
close all
clc

%Parametrizacion
letras = {'a', 'b', 'c'};
paso = 0.01;

for i = 1:length(letras)
    % Definición de t, x, y por casos
    switch letras{i}
        case 'a'
            t = 0:paso:2*pi;
            x = 2*cos(t); 
            y = 2*sin(t);
        case 'b'
            t = 0:paso:2*pi;
            x = cos(t); 
            y = sin(3*t);
        case 'c'
            t = 0:paso:2*pi;
            x = 1.5*cos(t) + 1*sin(20*t); 
            y = 1.5*sin(t) + 1*cos(20*t);
    end
    %Graficación en subplots (1 filas, 3 columnas)
    subplot(1, 3, i);
    plot(x, y, 'LineWidth', 1.5);
    title(['Caso ', letras{i}]);
    grid on;
    axis equal;
end
