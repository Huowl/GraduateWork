%% Plot main figure previos optimize model

mean_Z = mean(prev_simout.measBody.z.Data);
mean_velocity = mean(prev_simout.measBody.vX.Data);
CoT = CostOfTransport(prev_simout,actuatorType);

h = figure;
h.Position(3:4) = [1280 960];
set(h,'DefaultAxesFontSize',12,'DefaultAxesFontName','Times New Roman');
subplot(3,1,1)
hold on
grid on
plot(prev_simout.measBody.x.Time,prev_simout.measBody.x.Data, 'LineWidth',1);
ylabel('Координата по оси X, [м]');
subplot(3,1,2)
hold on
grid on
plot(prev_simout.measBody.z.Time,prev_simout.measBody.z.Data, 'LineWidth',1);
ylabel('Координата по оси Z, [м]');
subplot(3,1,3)
hold on
grid on
plot(prev_simout.measBody.vX.Time,prev_simout.measBody.vX.Data, 'LineWidth',1);
xlabel('Время, [с]');
ylabel('Скорость по оси X, [м/с]');