%% Parámetros del robot
L1 = 0.45;
L2 = 0.45;
L3 = 0.30;

%% Poses cartesianas — acordes al proyecto
x_in    = -1.0;   y_in    = 0.6;   theta_in  = 3*pi/4;
x_fin   =  1.0;   y_fin   = 0.6;   theta_fin = pi/4;

%% Planificación en espacio de trabajo con polinomio de quinto orden
tf   = 10;
dt = 0.1;
tsim = 0:dt:tf;

N = length(tsim);

x_P         = zeros(1, tf+1);
y_P         = zeros(1, tf+1);
theta_P     = zeros(1, tf+1);
theta_1_tray = zeros(1, tf+1);
theta_2_tray = zeros(1, tf+1);
theta_3_tray = zeros(1, tf+1);

for i = 1:N
    t = (i - 1)*dt;   % t va de 0 a tf
    lambda = (10/tf^3)*t^3 - (15/tf^4)*t^4 + (6/tf^5)*t^5;

    % Interpolación cartesiana con polinomio de quinto orden
    x_P(i)     = x_in     + lambda*(x_fin     - x_in);
    y_P(i)     = y_in     + lambda*(y_fin     - y_in);
    theta_P(i) = theta_in + lambda*(theta_fin - theta_in);

    % Paso 1: Desacoplamiento de la muñeca
    x3 = x_P(i) - L3*cos(theta_P(i));
    y3 = y_P(i) - L3*sin(theta_P(i));

    % Paso 2: Radio efectivo
    r = sqrt(x3^2 + y3^2);

    % Paso 3: Ángulo del codo (θ2) — alineado con el markdown
    beta  = acos(max(-1, min(1, (L1^2 + L2^2 - r^2) / (2*L1*L2))));
    theta_2_tray(i) = pi - beta;

    % Paso 4: Ángulo de la base (θ1)
    alpha = acos(max(-1, min(1, (L1^2 + r^2 - L2^2) / (2*L1*r))));
    psi   = atan2(y3, x3);
    theta_1_tray(i) = psi - alpha;

    % Paso 5: Orientación final (θ3)
    theta_3_tray(i) = theta_P(i) - theta_1_tray(i) - theta_2_tray(i);
end

%% Gráfica — trayectoria cartesiana
figure;
plot(tsim, x_P, 'r-x', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', 'x(t)');
hold on;
plot(tsim, y_P, 'g-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', 'y(t)');
plot(tsim, theta_P, 'b-*', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\theta(t)');
hold off;
legend
title('Trayectoria en espacio de trabajo — Polinomio de quinto orden')
xlabel('t [s]')
ylabel('m / rad')
grid on;

%% Gráfica — ángulos de juntas resultantes
figure;
plot(tsim, theta_1_tray, 'r-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\theta_1');
hold on;
plot(tsim, theta_2_tray, 'g-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\theta_2');
plot(tsim, theta_3_tray, 'b-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\theta_3');
hold off;
legend
title('Ángulos de juntas — Cinemática inversa sobre trayectoria cartesiana')
xlabel('t [s]')
ylabel('rad')
grid on;