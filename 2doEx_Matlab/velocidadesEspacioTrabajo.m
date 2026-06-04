%% Parámetros del robot
L_1 = 0.45;
L_2 = 0.45;
L_3 = 0.30;

%% Poses cartesianas — deben estar definidas (no comentadas)
x_in    = -1.0;   y_in    = 0.6;   theta_in  = 3*pi/4;
x_fin   =  1.0;   y_fin   = 0.6;   theta_fin = pi/4;

%% Verificación: theta_1_tray, theta_2_tray, theta_3_tray deben existir
% Si no se ejecutó el script anterior, calcularlos aquí
if ~exist('theta_1_tray', 'var')
    error('Ejecuta primero el script de trayectoria cartesiana para obtener theta_1_tray, theta_2_tray, theta_3_tray')
end

%% Velocidades en espacio de trabajo — polinomio de quinto orden (derivada)
tf   = 10;
dt = 0.1;
tsim = 0:dt:tf;

N = length(tsim);

x_P_v     = zeros(1, tf+1);
y_P_v     = zeros(1, tf+1);
theta_P_v = zeros(1, tf+1);
theta_1_v = zeros(1, tf+1);
theta_2_v = zeros(1, tf+1);
theta_3_v = zeros(1, tf+1);

for i = 1:N
    t = (i - 1)*dt;   % t va de 0 a tf — arranque y frenado en cero

    % Derivada del polinomio de quinto orden (velocidad cartesiana)
    lambda_v = (30/tf^3)*t^2 - (60/tf^4)*t^3 + (30/tf^5)*t^4;

    x_P_v(i)     = lambda_v * (x_fin     - x_in);
    y_P_v(i)     = lambda_v * (y_fin     - y_in);
    theta_P_v(i) = lambda_v * (theta_fin - theta_in);

    % Velocidades cartesianas del efector en este instante
    x_dot       = x_P_v(i);
    y_dot       = y_P_v(i);
    theta_dot_P = theta_P_v(i);

    % Ángulos de las juntas en este instante (del script de trayectoria)
    theta_O_1 = theta_1_tray(i);   % θ1 — articulación base
    theta_1_2 = theta_2_tray(i);   % θ2 — articulación codo
    theta_2_3 = theta_3_tray(i);   % θ3 — articulación muñeca

    % Velocidades de juntas por Jacobiano inverso
    theta_1_v(i) =   (x_dot     *  cos(theta_1_2 + theta_O_1)) / (L_1*sin(theta_1_2)) ...
                   + (y_dot     *  sin(theta_1_2 + theta_O_1)) / (L_1*sin(theta_1_2)) ...
                   + (L_3*theta_dot_P * sin(theta_2_3))        / (L_1*sin(theta_1_2));

    theta_2_v(i) = - (x_dot*(L_2*cos(theta_1_2 + theta_O_1) + L_1*cos(theta_O_1)))           / (L_1*L_2*sin(theta_1_2)) ...
                   - (y_dot*(L_2*sin(theta_1_2 + theta_O_1) + L_1*sin(theta_O_1)))           / (L_1*L_2*sin(theta_1_2)) ...
                   - (L_3*theta_dot_P*(L_1*sin(theta_1_2 + theta_2_3) + L_2*sin(theta_2_3))) / (L_1*L_2*sin(theta_1_2));

    theta_3_v(i) =   (theta_dot_P*(L_3*sin(theta_1_2 + theta_2_3) + L_2*sin(theta_1_2))) / (L_2*sin(theta_1_2)) ...
                   + (x_dot * cos(theta_O_1))                                             / (L_2*sin(theta_1_2)) ...
                   + (y_dot * sin(theta_O_1))                                             / (L_2*sin(theta_1_2));
end

%% Gráfica
figure;
plot(tsim, theta_1_v, 'r-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\dot{\theta}_1');
hold on;
plot(tsim, theta_2_v, 'g-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\dot{\theta}_2');
plot(tsim, theta_3_v, 'b-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\dot{\theta}_3');
hold off;
legend
title('Velocidades de juntas — Jacobiano inverso sobre trayectoria cartesiana')
xlabel('t [s]')
ylabel('rad/s')
grid on;