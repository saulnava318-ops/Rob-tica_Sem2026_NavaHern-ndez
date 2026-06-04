%% Cinemática Inversa — Robot SCARA 3R
%  Calcula los ángulos de las 4 posturas del cuadrado

clear; clc;

%% Parámetros del robot
L1 = 0.45;
L2 = 0.45;
L3 = 0.30;

%% Definición de las 4 poses [x, y, theta]
poses = [ -1.0,  0.4,  3*pi/4;   % Pose 1
           1.0,  0.4,    pi/4;   % Pose 2
           1.0, -0.4, -pi/4;   % Pose 3
           -1.0, -0.4, -3*pi/4];  % Pose 4

%% Cálculo de cinemática inversa para cada pose
resultados = zeros(4, 3);  % cada fila: [theta1, theta2, theta3]

for i = 1:4
    x_p     = poses(i, 1);
    y_p     = poses(i, 2);
    theta_p = poses(i, 3);

    % Paso 1: Desacoplamiento de la muñeca
    x3 = x_p - L3 * cos(theta_p);
    y3 = y_p  - L3 * sin(theta_p);

    % Paso 2: Radio efectivo
    r = sqrt(x3^2 + y3^2);

    % Paso 3: Ángulo del codo (theta_2)
    beta    = acos((L1^2 + L2^2 - r^2) / (2*L1*L2));
    theta_2 = pi - beta;

    % Paso 4: Ángulo de la base (theta_1)
    alpha   = acos((L1^2 + r^2 - L2^2) / (2*L1*r));
    psi     = atan2(y3, x3);
    theta_1 = psi - alpha;

    % Paso 5: Orientación final (theta_3)
    theta_3 = theta_p - theta_1 - theta_2;

    resultados(i, :) = [theta_1, theta_2, theta_3];
end

%% Mostrar los 24 valores entre pares de poses
pares = [1, 2;
    2, 3;
    3, 4;
    4, 1];  % pares de poses: inicio → fin

for k = 1:4
    i = pares(k, 1);
    j = pares(k, 2);

    fprintf('\n════════════════════════════════════════\n');
    fprintf(' Segmento %d: Pose %d → Pose %d\n', k, i, j);
    fprintf('════════════════════════════════════════\n');

    fprintf(' [Pose %d — Inicio]\n', i);
    fprintf('   theta_1 = %8.4f rad\n', resultados(i, 1));
    fprintf('   theta_2 = %8.4f rad\n', resultados(i, 2));
    fprintf('   theta_3 = %8.4f rad\n', resultados(i, 3));

    fprintf(' [Pose %d — Fin]\n', j);
    fprintf('   theta_1 = %8.4f rad\n', resultados(j, 1));
    fprintf('   theta_2 = %8.4f rad\n', resultados(j, 2));
    fprintf('   theta_3 = %8.4f rad\n', resultados(j, 3));
end

%% Tabla resumen de los 24 valores
fprintf('\n\n════════════════════════════════════════════════════════\n');
fprintf(' RESUMEN — 24 valores de ángulos\n');
fprintf('════════════════════════════════════════════════════════\n');
fprintf(' %-8s %-10s %-10s %-10s\n', 'Pose', 'theta_1', 'theta_2', 'theta_3');
fprintf('────────────────────────────────────────────────────────\n');
for i = 1:4
    fprintf(' Pose %-4d %10.4f %10.4f %10.4f\n', i, ...
        resultados(i,1), resultados(i,2), resultados(i,3));
end
fprintf('════════════════════════════════════════════════════════\n');