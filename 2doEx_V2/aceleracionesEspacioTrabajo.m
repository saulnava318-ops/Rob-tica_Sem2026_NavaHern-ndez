%% Parámetros del robot
L_1 = 0.45;
L_2 = 0.45;
L_3 = 0.30;

%% Poses cartesianas
x_in    = -1.0;   y_in    = 0.6;   theta_in  = 3*pi/4;
x_fin   =  1.0;   y_fin   = 0.6;   theta_fin = pi/4;

%% Verificación de dependencias
if ~exist('theta_1_tray', 'var') || ~exist('theta_1_v', 'var')
    error('Ejecuta primero los scripts de trayectoria y velocidad.')
end

%% Aceleraciones en espacio de trabajo
tf   = 10;
dt = 0.1;
tsim = 0:dt:tf;

N = length(tsim);

x_P_a     = zeros(1, tf+1);
y_P_a     = zeros(1, tf+1);
theta_P_a = zeros(1, tf+1);
theta_1_a = zeros(1, tf+1);
theta_2_a = zeros(1, tf+1);
theta_3_a = zeros(1, tf+1);

for i = 1:N
    t = (i - 1)*dt;

    % Segunda derivada del polinomio quíntico (aceleración cartesiana)
    lambda_a = (60/tf^3)*t - (180/tf^4)*t^2 + (120/tf^5)*t^3;

    x_P_a(i)     = lambda_a * (x_fin     - x_in);
    y_P_a(i)     = lambda_a * (y_fin     - y_in);
    theta_P_a(i) = lambda_a * (theta_fin - theta_in);

    % Aceleraciones cartesianas del efector
    x_ddot       = x_P_a(i);
    y_ddot       = y_P_a(i);
    theta_ddot_P = theta_P_a(i);

    % Ángulos y velocidades de juntas en este instante
    theta_O_1  = theta_1_tray(i);
    theta_1_2  = theta_2_tray(i);
    theta_2_3  = theta_3_tray(i);
    dtheta_1   = theta_1_v(i);
    dtheta_2   = theta_2_v(i);
    dtheta_3   = theta_3_v(i);

    % Velocidades cartesianas en este instante (para término J_dot*q_dot)
    lambda_v   = (30/tf^3)*t^2 - (60/tf^4)*t^3 + (30/tf^5)*t^4;
    x_dot      = lambda_v * (x_fin - x_in);
    y_dot      = lambda_v * (y_fin - y_in);
    theta_dot_P = lambda_v * (theta_fin - theta_in);

    % Término J_dot * q_dot — corrección por cambio del Jacobiano
    % Componente en x — segunda derivada de la cinemática directa
    Jd_q_x = - L_1 * sin(theta_O_1) * dtheta_1^2 ...
              - L_2 * sin(theta_O_1 + theta_1_2) * (dtheta_1 + dtheta_2)^2 ...
              - L_3 * sin(theta_O_1 + theta_1_2 + theta_2_3) * (dtheta_1 + dtheta_2 + dtheta_3)^2;
    
    % Componente en y
    Jd_q_y =   L_1 * cos(theta_O_1) * dtheta_1^2 ...
              + L_2 * cos(theta_O_1 + theta_1_2) * (dtheta_1 + dtheta_2)^2 ...
              + L_3 * cos(theta_O_1 + theta_1_2 + theta_2_3) * (dtheta_1 + dtheta_2 + dtheta_3)^2;
    
    % Componente en theta — sigue siendo cero
    Jd_q_th = 0;

    % Vector de aceleración cartesiana corregida: x_ddot - J_dot*q_dot
    ax_corr  = x_ddot     - Jd_q_x;
    ay_corr  = y_ddot     - Jd_q_y;
    ath_corr = theta_ddot_P - Jd_q_th;

    % Aceleraciones de juntas por Jacobiano inverso: q_ddot = J^-1 * (x_ddot - J_dot*q_dot)
    theta_1_a(i) =   (ax_corr  *  cos(theta_1_2 + theta_O_1)) / (L_1*sin(theta_1_2)) ...
                   + (ay_corr  *  sin(theta_1_2 + theta_O_1)) / (L_1*sin(theta_1_2)) ...
                   + (L_3*ath_corr * sin(theta_2_3))          / (L_1*sin(theta_1_2));

    theta_2_a(i) = - (ax_corr*(L_2*cos(theta_1_2 + theta_O_1) + L_1*cos(theta_O_1)))            / (L_1*L_2*sin(theta_1_2)) ...
                   - (ay_corr*(L_2*sin(theta_1_2 + theta_O_1) + L_1*sin(theta_O_1)))            / (L_1*L_2*sin(theta_1_2)) ...
                   - (L_3*ath_corr*(L_1*sin(theta_1_2 + theta_2_3) + L_2*sin(theta_2_3)))       / (L_1*L_2*sin(theta_1_2));

    theta_3_a(i) =   (ath_corr*(L_3*sin(theta_1_2 + theta_2_3) + L_2*sin(theta_1_2))) / (L_2*sin(theta_1_2)) ...
                   + (ax_corr * cos(theta_O_1))                                        / (L_2*sin(theta_1_2)) ...
                   + (ay_corr * sin(theta_O_1))                                        / (L_2*sin(theta_1_2));
end

%% Gráfica
figure;
plot(tsim, theta_1_a, 'r-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\ddot{\theta}_1');
hold on;
plot(tsim, theta_2_a, 'g-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\ddot{\theta}_2');
plot(tsim, theta_3_a, 'b-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\ddot{\theta}_3');
hold off;
legend
title('Aceleraciones de juntas — Jacobiano inverso sobre trayectoria cartesiana')
xlabel('t [s]')
ylabel('rad/s²')
grid on;