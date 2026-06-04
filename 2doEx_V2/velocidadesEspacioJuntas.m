%% Velocidades de las juntas
tf   = 10;
tsim = 0:1:tf;

for i = 1:tf+1
    t = i - 1;
    lambda_v = (30/tf^3)*t^2 - (60/tf^4)*t^3 + (30/tf^5)*t^4;
    theta_1_P_v(i) = lambda_v * (theta_1_fin - theta_1_in);
    theta_2_P_v(i) = lambda_v * (theta_2_fin - theta_2_in);
    theta_3_P_v(i) = lambda_v * (theta_3_fin - theta_3_in);
end

figure;
plot(tsim, theta_1_P_v, 'r-x',  'LineWidth', 3, 'MarkerSize', 12, 'DisplayName', '\dot{\theta}_1');
hold on;
plot(tsim, theta_2_P_v, 'g-o', 'LineWidth', 3, 'MarkerSize', 12, 'DisplayName', '\dot{\theta}_2');
plot(tsim, theta_3_P_v, 'b-*', 'LineWidth', 3, 'MarkerSize', 12, 'DisplayName', '\dot{\theta}_3');
hold off;
legend
title('Velocidades en espacio de las juntas — Polinomio de quinto orden')
xlabel('t [s]')
ylabel('rad/s')
grid on;

