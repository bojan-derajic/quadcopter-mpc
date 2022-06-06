close all; clc;

Ft = out.U.signals.values(:, 1);
Mx = out.U.signals.values(:, 2);
My = out.U.signals.values(:, 3);
Mz = out.U.signals.values(:, 4);

x = out.Y.signals.values(:, 1);
y = out.Y.signals.values(:, 2);
z = out.Y.signals.values(:, 3);
yaw = out.Y.signals.values(:, 4);

if size(out.R.signals.values, 1) == 1
    out.R.signals.values = squeeze(out.R.signals.values)';
end

r_x = out.R.signals.values(:, 1);
r_y = out.R.signals.values(:, 2);
r_z = out.R.signals.values(:, 3);
r_yaw = out.R.signals.values(:, 4);

figure('Position', [10 50 650 500]);
plot3(r_x, r_y, r_z, 'r--', 'linewidth', 1.5);
hold on;
grid on;
plot3(x, y, z, 'b', 'linewidth', 2);
axis('equal');
xlabel('x');
ylabel('y');
zlabel('z');
legend({'Reference', 'True position'})

figure('Position', [10 0 500 1000]);

subplot(4, 1, 1);
plot(out.R.time, r_x, 'r--', 'linewidth', 1);
hold on;
grid on;
plot(out.Y.time, x, 'b', 'linewidth', 1);
xlabel('t [s]');
ylabel('x [m]');
legend({'Reference', 'True Position'}, 'location', 'northwest');

subplot(4, 1, 2);
plot(out.R.time, r_y, 'r--', 'linewidth', 1);
hold on;
grid on;
plot(out.Y.time, y, 'b', 'linewidth', 1);
xlabel('t [s]');
ylabel('y [m]');
legend({'Reference', 'True Position'}, 'location', 'northwest');

subplot(4, 1, 3);
plot(out.R.time, r_z, 'r--', 'linewidth', 1);
hold on;
grid on;
plot(out.Y.time, z, 'b', 'linewidth', 1);
xlabel('t [s]');
ylabel('z [m]');
legend({'Reference', 'True Position'}, 'location', 'southwest');

subplot(4, 1, 4);
plot(out.R.time, r_yaw, 'r--', 'linewidth', 1);
hold on;
grid on;
plot(out.Y.time, yaw, 'b', 'linewidth', 1);
xlabel('t [s]');
ylabel('yaw [rad]');
legend({'Reference', 'True angle'}, 'location', 'northwest');

figure('Position', [10 0 500 1000]);

subplot(4, 1, 1);
plot(out.U.time, Ft, 'b', 'linewidth', 1);
grid on;
xlabel('t [s]');
ylabel('Ft [N]');

subplot(4, 1, 2);
plot(out.U.time, Mx, 'b', 'linewidth', 1);
grid on;
xlabel('t [s]');
ylabel('Mx [Nm]');

subplot(4, 1, 3);
plot(out.U.time, My, 'b', 'linewidth', 1);
grid on;
xlabel('t [s]');
ylabel('My [Nm]');

subplot(4, 1, 4);
plot(out.U.time, Mz, 'b', 'linewidth', 1);
grid on;
xlabel('t [s]');
ylabel('Mz [Nm]');

E = out.R.signals.values - out.Y.signals.values;
MSE = mean(E.^(2), 1);
disp('Mean Squared Error:')
disp(MSE);

MP = mean(out.U.signals.values.^(2), 1);
disp('Mean Control Signal Effort');
disp(MP)
