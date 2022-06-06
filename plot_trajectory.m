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

figure('Position', [10 50 750 500]);
plot3(r_x, r_y, r_z, 'r--', 'linewidth', 1.5);
hold on;
grid on;
plot3(x, y, z, 'b', 'linewidth', 2);
axis('equal');
xlabel('x');
ylabel('y');
zlabel('z');
legend({'Reference', 'True position'})

figure('Position', [10 50 1300 550]);

subplot(2, 2, 1);
plot(out.R.time, r_x, 'r--', 'linewidth', 1);
hold on;
grid on;
plot(out.Y.time, x, 'b', 'linewidth', 1);
xlabel('t [s]');
ylabel('x [m]');
legend({'Reference', 'True Position'}, 'location', 'northwest');

subplot(2, 2, 2);
plot(out.R.time, r_y, 'r--', 'linewidth', 1);
hold on;
grid on;
plot(out.Y.time, y, 'b', 'linewidth', 1);
xlabel('t [s]');
ylabel('y [m]');
legend({'Reference', 'True Position'}, 'location', 'northwest');

subplot(2, 2, 3);
plot(out.R.time, r_z, 'r--', 'linewidth', 1);
hold on;
grid on;
plot(out.Y.time, z, 'b', 'linewidth', 1);
xlabel('t [s]');
ylabel('z [m]');
legend({'Reference', 'True Position'}, 'location', 'southwest');

subplot(2, 2, 4);
plot(out.R.time, r_yaw, 'r--', 'linewidth', 1);
hold on;
grid on;
plot(out.Y.time, yaw, 'b', 'linewidth', 1);
xlabel('t [s]');
ylabel('yaw [rad]');
legend({'Reference', 'True angle'}, 'location', 'northwest');

E = out.R.signals.values - out.Y.signals.values;
E = sqrt(sum(E.^(2), 2));

figure();
plot(out.R.time, E, 'linewidth', 1);
grid on;
xlabel('t [s]');
ylabel('||e||');