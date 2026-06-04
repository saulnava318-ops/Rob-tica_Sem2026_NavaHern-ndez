%% Parámetros del robot
L1 = 0.45;
L2 = 0.45;
L3 = 0.30;

%% Poses cartesianas
x_in    = -1.0;   y_in    = 0.6;   theta_in  = 3*pi/4;
x_fin   =  1.0;   y_fin   = 0.6;   theta_fin = pi/4;

%% Cinemática inversa — Posicion inicial
x3_in  = x_in  - L3*cos(theta_in);
y3_in  = y_in  - L3*sin(theta_in);
r_in   = sqrt(x3_in^2 + y3_in^2);
beta_in    = acos((L1^2 + L2^2 - r_in^2) / (2*L1*L2));
theta_1_in = atan2(y3_in, x3_in) - acos((L1^2 + r_in^2 - L2^2) / (2*L1*r_in));
theta_2_in = pi - beta_in;
theta_3_in = theta_in - theta_1_in - theta_2_in;

%% Cinemática inversa — Posicion final
x3_fin = x_fin - L3*cos(theta_fin);
y3_fin = y_fin - L3*sin(theta_fin);
r_fin  = sqrt(x3_fin^2 + y3_fin^2);
beta_fin   = acos((L1^2 + L2^2 - r_fin^2) / (2*L1*L2));
theta_1_fin = atan2(y3_fin, x3_fin) - acos((L1^2 + r_fin^2 - L2^2) / (2*L1*r_fin));
theta_2_fin = pi - beta_fin;
theta_3_fin = theta_fin - theta_1_fin - theta_2_fin;

%% Planificación con polinomio de quinto orden
tf   = 10;                      % tiempo final [s]
tsim = 0:1:tf;                  % vector de tiempo (0,1,2,...,10)

theta_1_P = zeros(1, tf+1);
theta_2_P = zeros(1, tf+1);
theta_3_P = zeros(1, tf+1);

for i = 1:tf+1
    t = i - 1;
    lambda = (10/tf^3)*t^3 - (15/tf^4)*t^4 + (6/tf^5)*t^5;

    theta_1_P(i) = theta_1_in + lambda*(theta_1_fin - theta_1_in);
    theta_2_P(i) = theta_2_in + lambda*(theta_2_fin - theta_2_in);
    theta_3_P(i) = theta_3_in + lambda*(theta_3_fin - theta_3_in);
end

%% Mostrar valores calculados
fprintf('theta_1_in = %.4f rad,  theta_1_fin = %.4f rad\n', theta_1_in, theta_1_fin)
fprintf('theta_2_in = %.4f rad,  theta_2_fin = %.4f rad\n', theta_2_in, theta_2_fin)
fprintf('theta_3_in = %.4f rad,  theta_3_fin = %.4f rad\n', theta_3_in, theta_3_fin)

%% Gráfica
figure;
hold on;
plot(tsim, theta_1_P, 'r-x', 'LineWidth', 3, 'MarkerSize', 10, 'DisplayName', '\theta_1');
plot(tsim, theta_2_P, 'g-o', 'LineWidth', 3, 'MarkerSize', 10, 'DisplayName', '\theta_2');
plot(tsim, theta_3_P, 'b-*', 'LineWidth', 3, 'MarkerSize', 10, 'DisplayName', '\theta_3');
hold off;
legend
title('Planeación en espacio de las juntas — Polinomio de quinto orden')
xlabel('t [s]')
ylabel('rad')
grid on;