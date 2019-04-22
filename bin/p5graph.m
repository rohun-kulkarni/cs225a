% Problem 5 graph
ctl4data = importdata("controller4.txt", ',');
ctl5data = importdata("controller5.txt", ',');
figure();
plot(ctl4data(:,2),"--r", "LineWidth", 2.0);
hold on;
plot(ctl4data(:,3),"--b", "LineWidth", 2.0);
plot(ctl4data(:, 4),"--g", "LineWidth", 2.0);
plot(ctl5data(:,2),"-r", "LineWidth", 2.0);
hold on;
plot(ctl5data(:,3),"-b","LineWidth", 2.0);
plot(ctl5data(:, 4),"-g","LineWidth", 2.0);
xlabel("SimulationStep" );
ylabel("Joint Angle");
title("Problem 5:Joint angles for Joints 1, 3, 4 with PD control, Kv = 50");
legend("P4 Q1", "P4 Q3", "P4 Q4", "P5 Q1", "P5 Q3", "P5 Q4");
