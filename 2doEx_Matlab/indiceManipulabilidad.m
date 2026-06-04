%% Índice de manipulabilidad de Yoshikawa
% μ = |det(J)| = |L1·L2·sin(θ2)·L3|
% Indica qué tan lejos está el robot de una singularidad.
% μ = 0 → singularidad (robot completamente extendido o doblado)
% μ máximo → mejor capacidad de movimiento en todas direcciones

w = zeros(1, tf+1);

for i = 1:tf+1
    w(i) = abs(L_1 * L_2 * L_3 * sin(theta_2_tray(i)));
end

figure;
plot(tsim, w, 'r-+', 'LineWidth', 3, 'MarkerSize', 12, 'DisplayName', '\mu');
hold on;
% Línea de referencia en cero — zona de singularidad
yline(0, 'w--', 'LineWidth', 1.5, 'DisplayName', 'Singularidad');
hold off;
legend
title('Índice de manipulabilidad — \mu = |L_1 L_2 L_3 \sin(\theta_2)|')
xlabel('t [s]')
ylabel('\mu')
grid on;

fprintf('Manipulabilidad mínima: %.4f (t = %d s)\n', min(w), find(w == min(w)) - 1)
fprintf('Manipulabilidad máxima: %.4f (t = %d s)\n', max(w), find(w == max(w)) - 1)