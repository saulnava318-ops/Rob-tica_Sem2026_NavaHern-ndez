
for i=1:tf+1
    
    theta_1 = theta_1_tray(i);
    theta_2 = theta_2_tray(i);
    theta_3 = theta_3_tray(i);
    theta_1_vc = theta_1_v(i);
    theta_2_vc = theta_2_v(i);
    theta_3_vc = theta_3_v(i);
    theta_1_ac = theta_1_a(i);
    theta_2_ac = theta_2_a(i);
    theta_3_ac = theta_3_a(i);
    sigma_19 = sin(theta_2+theta_1)/2;
    sigma_18 = sin(theta_1+theta_2+theta_3);
    sigma_17 = cos(theta_2+theta_1)/2;
    sigma_16 = cos(theta_1+theta_2+theta_3);
    sigma_15 = theta_2_vc*(sigma_18/8 + sigma_19);
    sigma_14 = sigma_18/8 + sigma_19 + sin(theta_1)/2;
    sigma_13 = theta_2_vc *(sigma_16/8 +sigma_17);
    sigma_12 = sigma_16/8 +sigma_17 +cos(theta_1)/2;
    sigma_11 = theta_3_vc*sigma_18/8 + theta_1_vc*sigma_14 +sigma_15;
    sigma_10 = sigma_13 + theta_3_vc*sigma_16/8 +theta_1_vc*sigma_12;
    sigma_9 = ((sigma_16/8 + sigma_17)*sigma_11)/2;
    sigma_8 = ((sigma_18/8 + sigma_19)*sigma_10)/2;
    sigma_7 = (sigma_16*(sigma_16/8 + sigma_17))/16 + (sigma_18*(sigma_18/8 + sigma_19))/16 + 1;
    sigma_6 = sigma_16*sigma_12/16 + sigma_18*sigma_14 + 1;
    sigma_5 = theta_2_vc*sigma_16/8 + theta_3_vc*sigma_16/8 + theta_1_vc*sigma_16/8;
    sigma_4 = theta_2_vc*sigma_18/8 + theta_3_vc*sigma_18/8 + theta_1_vc*sigma_18/8;
    sigma_3 = cos(theta_2)/8 + ((sigma_16/8 + sigma_17)*sigma_12)/2 + ((sigma_18/8 + sigma_19)*sigma_14)/2 + 33/16;
    sigma_2 = sigma_13 +theta_1_vc*(sigma_16/8 + sigma_17) + theta_3_vc*sigma_16/8;
    sigma_1 = theta_3_vc*sigma_18/8 + sigma_15 +theta_1_vc*(sigma_18/8 + sigma_19);
    tao_1_c(i) = theta_1_ac*(cos(theta_2)/4 + sigma_12^2/2 + sigma_14^2/2 + 27/8) - theta_3_vc* (sigma_18*sigma_10/16 - sigma_16*sigma_11/16 + sigma_12*sigma_4/2 - sigma_5*sigma_14/2) + theta_2_ac*sigma_3 + theta_3_ac*sigma_6 - theta_2_vc*(sigma_8 - sigma_14*sigma_2/2 + sigma_1*sigma_12/2 - sigma_9 + theta_2_vc*sin(theta_2)/8 + theta_1_vc*sin(theta_2)/4);
    tao_2_c(i) = theta_1_vc^2*sin(theta_2)/8 - theta_3_vc*(sigma_18*sigma_10 - ((sigma_18/8 + sigma_19)*sigma_5)/2 + ((sigma_16/8 + sigma_17)*sigma_4)/2 - sigma_16*sigma_11/16) + theta_2_ac*sigma_7 + sigma_10*sigma_1/2 + theta_1_ac*sigma_3 - sigma_11*sigma_2/2 + theta_2_ac*(((sigma_16 + sigma_17)^2)/2 + ((sigma_18 + sigma_19)^2)/2 + 33/16) - theta_2_vc*((sigma_16/8 + sigma_17)*sigma_1/2 + (sigma_18/8 + sigma_19)*sigma_2/2 + sigma_8 - sigma_9 + theta_1_vc*sin(theta_2)/8) + theta_2_vc*theta_1_vc*sin(theta_2)/8;
    tao_3_c(i) = theta_3_ac*(sigma_16^2/128 + sigma_18^2/128 + 1 ) - sigma_5*sigma_11/2 + theta_2_ac*sigma_7 - theta_3_vc*(sigma_18*sigma_10/16 - sigma_18*sigma_5/16 - sigma_16*sigma_11/16 + sigma_16*sigma_4/16) + theta_2_vc*(sigma_18*sigma_2/16 - sigma_18*sigma_10/16 - sigma_16*sigma_1/16 + sigma_16*sigma_11/16) + theta_1_ac*sigma_6 + sigma_10*sigma_4/2;
    pot_1(i) = abs(tao_1_c(i)*theta_1_vc);
    pot_2(i) = abs(tao_2_c(i)*theta_2_vc);
    pot_3(i) = abs(tao_3_c(i)*theta_3_vc);
    pot_total(i) = pot_1(i) + pot_2(i) + pot_3(i);
    
end


figure;
plot(tsim, tao_1_c, 'r-+', 'LineWidth',3, 'MarkerSize',10,'DisplayName','{\tau}_1_c');
hold on;
plot(tsim, tao_2_c, 'g-o','LineWidth',3, 'MarkerSize',10,'DisplayName','{\tau}_2_c');
plot(tsim, tao_3_c, 'b-*','LineWidth',3, 'MarkerSize',10,'DisplayName','{\tau}_3_c');
hold off;
legend
title('Torques producidos por los motores - Planeación en el Espacio de Trabajo')
xlabel('t[s]')
ylabel('N*m')
grid on;



figure;
plot(tsim, pot_1, 'r-+', 'LineWidth',3, 'MarkerSize',10,'DisplayName','{\P}_1');
hold on;
plot(tsim, pot_2, 'g-o', 'LineWidth',3, 'MarkerSize',10,'DisplayName','{\P}_2');
plot(tsim, pot_3, 'b-*', 'LineWidth',3, 'MarkerSize',10,'DisplayName','{\P}_3');
plot(tsim, pot_total, '-x','Color', [0.5,0,0.5], 'LineWidth',3, 'MarkerSize',10,'DisplayName','{\P}_t');
hold off;
legend
title('Potencia de los motores - Planeación en el Espacio de Trabajo')
xlabel('t[s]')
ylabel('W')
grid on;