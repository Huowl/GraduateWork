%% Plot main figure previos optimize model

mean_Z = mean(prev_simout.measBody.z.Data);
mean_velocity = mean(prev_simout.measBody.vX.Data);
CoT = CostOfTransport(prev_simout,actuatorType);

figure(1)
subplot(3,1,1)
hold on
grid on
plot(prev_simout.measBody.x.Time,prev_simout.measBody.x.Data, 'LineWidth',1);
title('Simulation Output Comparisons');
ylabel('$X$ [m]','Interpreter','latex');
subplot(3,1,2)
hold on
grid on
plot(prev_simout.measBody.z.Time,prev_simout.measBody.z.Data, 'LineWidth',1);
ylabel('$Z$ [m]','Interpreter','latex');
subplot(3,1,3)
hold on
grid on
plot(prev_simout.measBody.vX.Time,prev_simout.measBody.vX.Data, 'LineWidth',1);
xlabel('Time [s]');
ylabel('$\nu_x$ [m/s]','Interpreter','latex');