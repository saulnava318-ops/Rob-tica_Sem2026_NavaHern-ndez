% Definicion de los puntos
% Pose inicial
x_in = 0.6;
y_in = 0.5;
theta_in = 0; 
% Pose final
x_fin = 0.6;
y_fin = -0.5;
theta_fin = -pi/4;
% Cálculo de la solución de la primer postura

% Definición de los parámetros del robot
L1 = 0.45; % Longitud del primer eslabón
L2 = 0.45; % Longitud del segundo eslabón
L3 = 0.3; % Longitud del tercer eslabón

% Planteamiento de la solucion

x_3 = x_in - L3*cos(theta_in);
y_3 = y_in - L3*sin(theta_in);

% Solucion por le metodo geometrico

theta_2_in = acos((x_3^2+y_3^2-L1^2-L2^2)/(2*L1*L2))
beta = atan2(y_3,x_3);

psi = acos((x_3^2+y_3^2+L1^2-L2^2)/(2*L1*sqrt(x_3^2+y_3^2)));

% theta_1_in = beta + psi

 theta_1_in = beta - psi

% theta_1_in = beta - psi

theta_3_in = theta_in -theta_1_in - theta_2_in
%% Solucion para el punto final

x_3 = x_fin - L3*cos(theta_fin);
y_3 = y_fin - L3*sin(theta_fin);

% Solucion por le metodo geometrico

theta_2_fin = acos((x_3^2+y_3^2-L1^2-L2^2)/(2*L1*L2))

beta = atan2(y_3,x_3);

psi = acos((x_3^2+y_3^2+L1^2-L2^2)/(2*L1*sqrt(x_3^2+y_3^2)));

% theta_1_fin = beta + psi

theta_1_fin = beta - psi
theta_3_fin = theta_fin -theta_1_fin - theta_2_fin

clear; clc;

%% Parámetros del robot (URDF)
L1 = 0.45;
L2 = 0.45;
L3 = 0.30;

% Límites de las articulaciones (del URDF: -pi a pi)
theta1_range = linspace(-pi, pi, 150);
theta2_range = linspace(-pi, pi, 150);
theta3_range = linspace(-pi, pi, 60);

%% Cinemática directa — barrido de todos los ángulos
% Pre-reservar espacio (opcional pero más rápido)
n = length(theta1_range) * length(theta2_range) * length(theta3_range);
px = zeros(1, n);
py = zeros(1, n);

idx = 1;
for t1 = theta1_range
    for t2 = theta2_range
        % Posición del joint 3
        x3 = L1*cos(t1) + L2*cos(t1 + t2);
        y3 = L1*sin(t1) + L2*sin(t1 + t2);

        for t3 = theta3_range
            % Orientación acumulada del efector
            theta_ef = t1 + t2 + t3;

            % Posición del efector final
            px(idx) = x3 + L3*cos(theta_ef);
            py(idx) = y3 + L3*sin(theta_ef);
            idx = idx + 1;
        end
    end
end

%% Visualización
figure;
plot(px, py, '.', 'MarkerSize', 1, 'Color', [0.2 0.5 1 0.3]);
hold on;

% Origen de la base
plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

% Círculos de referencia
r_max = L1 + L2 + L3;
r_min = abs(L1 - L2);
ang = linspace(0, 2*pi, 360);
plot(r_max*cos(ang), r_max*sin(ang), 'r--', 'LineWidth', 1.5);
plot(r_min*cos(ang), r_min*sin(ang), 'g--', 'LineWidth', 1.5);

axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Workspace del efector final — SCARA 3R');
legend('Puntos alcanzables', 'Base', ...
       sprintf('R_{max} = %.2f m', r_max), ...
       sprintf('R_{min} = %.2f m', r_min), ...
       'Location', 'bestoutside');

% Línea de trayectoria del efector
x_tray = linspace(-1, 1, 100);
y_tray = 0.6 * ones(1, 100);
plot(x_tray, y_tray, 'y-', 'LineWidth', 2.5, 'DisplayName', 'Trayectoria efector');

% Puntos de inicio y fin
plot(-1, 0.6, 'y^', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'P1 inicio');
plot( 1, 0.6, 'ys', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'P2 fin');