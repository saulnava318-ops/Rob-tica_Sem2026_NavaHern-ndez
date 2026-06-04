%% Aceleraciones de las juntas
for i = 1:tf+1
    t = i - 1;
    lambda_a = (60/tf^3)*t - (180/tf^4)*t^2 + (120/tf^5)*t^3;
    theta_1_P_a(i) = lambda_a * (theta_1_fin - theta_1_in);
    theta_2_P_a(i) = lambda_a * (theta_2_fin - theta_2_in);
    theta_3_P_a(i) = lambda_a * (theta_3_fin - theta_3_in);
end

figure;
plot(tsim, theta_1_P_a, 'r-x',  'LineWidth', 3, 'MarkerSize', 10, 'DisplayName', '\ddot{\theta}_1');
hold on;
plot(tsim, theta_2_P_a, 'g-o', 'LineWidth', 3, 'MarkerSize', 10, 'DisplayName', '\ddot{\theta}_2');
plot(tsim, theta_3_P_a, 'b-*', 'LineWidth', 3, 'MarkerSize', 10, 'DisplayName', '\ddot{\theta}_3');
hold off;
legend
title('Aceleraciones en espacio de las juntas — Polinomio de quinto orden')
xlabel('t [s]')
ylabel('rad/s²')
grid on;