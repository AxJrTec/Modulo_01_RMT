clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = 12;             
ts = 0.1;            
t = 0: ts: tf;       
N = length(t);       

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = zeros(1,N+1);  
y1 = zeros(1,N+1);  
phi = zeros(1,N+1); 

x1(1) = -6;              
y1(1) = -12;              
phi(1) = 0.4636;             

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hx = zeros(1,N+1);  
hy = zeros(1,N+1);  

hx(1) = x1(1);
hy(1) = y1(1);

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%

u = zeros(1,N); 
w = zeros(1,N); 

for k = 1:N
    
    tk = t(k);
    
    % -------- TRAMO 1 --------
    % Función y(t) = 2(t - 6)
    % Función x(t) = t - 6
    % t = [0 a 5]

    if tk <= 5
       
        dx = 1; 
        dy = 2;
        a = dy / dx;
        da = 0;
        
        u(k) = sqrt(dx^2 + dy^2);
        w(k) = (da / (1 + a^2));
        
            
    % -------- TRAMO 2 --------
    % Función y(t) = 2(t - 6) + 1
    % Función x(t) = t - 6
    % t = [5 a 7]

    elseif tk < 7
        
        dx = 1; 
        dy = 2;
        a = dy / dx;
        da = 0;
        
        u(k) = sqrt(dx^2 + dy^2);
        w(k) = (da / (1 + a^2));
    
    % -------- TRAMO 3 --------
    % Función y(t) = -(t - 6) + 4
    % Función x(t) = t - 6
    % t = [7 a 10]

    elseif tk < 10
        
        dx = 1;
        dy = -1;
        a = dy / dx;
        da = 0;
        
        u(k) = sqrt(dx^2 + dy^2);
        w(k) = (da / (1 + a^2));
            
    % -------- TRAMO 4 --------
    % Función y(t) = (t - 6) - 1 
    % Función x(t) = t - 6
    % t = [10 a infinito]
        
    else
        
        dx = 1;
        dy = 1;
        a = dy / dx;
        da = 0;

        u(k) = sqrt(dx^2 + dy^2);
        w(k) = (da / (1 + a^2));
        
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N 
    
    phi(k+1)=phi(k)+w(k)*ts;
    
    xp1=u(k)*cos(phi(k+1)); 
    yp1=u(k)*sin(phi(k+1));

    x1(k+1)=x1(k) + xp1*ts; 
    y1(k+1)=y1(k) + yp1*ts; 
    
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scene=figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen=get(0,'ScreenSize');
set(scene,'position',sizeScreen);
camlight('headlight');
axis equal;
grid on;
box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');

view([25 25]);
axis([-10 15 -20 20 0 2]);
scale = 1;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);

step=1;

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRAFICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

graph=figure;
set(graph,'position',sizeScreen);

subplot(211)
plot(t,u,'b','LineWidth',2),grid on
xlabel('Tiempo [s]'),ylabel('m/s'),legend('u')

subplot(212)
plot(t,w,'r','LineWidth',2),grid on
xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('w')