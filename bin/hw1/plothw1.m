ctl1data = importdata("controller1.txt", ',');
ctl2data = importdata("controller2.txt", ',');
ctl3data = importdata("controller3.txt", ',');
ctl4data = importdata("controller3.txt", ',');

% Problem 1 Graph
figure();
plot(ctl1data(:,2), "LineWidth", 2.0);
hold on;
plot(ctl1data(:,3), "LineWidth", 2.0);
plot(ctl1data(:, 4), "LineWidth", 2.0);
xlabel("SimulationStep" );
ylabel("Joint Angle");
title("Problem 1:Joint angles for Joints 1, 3, 4 with PD control, Kv = 50");
legend("Q1", "Q3", "Q4");
save("ctldataAll2.mat");


% Problem 2 graph