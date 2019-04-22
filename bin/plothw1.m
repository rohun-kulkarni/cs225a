close all;
ctl1data = importdata("controller1.txt", ',');
ctl2data = importdata("controller2.txt", ',');
ctl3data = importdata("controller3.txt", ',');
ctl4data = importdata("controller4.txt", ',');

% Problem 1 Graph
figure();
plot(ctl1data(:,2),"-r", "LineWidth", 2.0);
hold on;
plot(ctl1data(:,3),"-b", "LineWidth", 2.0);
plot(ctl1data(:, 4),"-g", "LineWidth", 2.0);
xlabel("SimulationStep" );
ylabel("Joint Angle");
title("Problem 1:Joint angles for Joints 1, 3, 4 with PD control, Kv = 50");
legend("Q1", "Q3", "Q4");
hold off;

% Problem 2 graph
figure();
plot(ctl1data(:,2),"--r", "LineWidth", 2.0);
hold on;
plot(ctl1data(:,3),"--b", "LineWidth", 2.0);
plot(ctl1data(:, 4),"--g", "LineWidth", 2.0);
hold on; 
plot(ctl2data(:,2),"-r", "LineWidth", 2.0);
hold on;
plot(ctl2data(:,3),"-b","LineWidth", 2.0);
plot(ctl2data(:, 4),"-g","LineWidth", 2.0);
xlabel("SimulationStep" );
ylabel("Joint Angle");
title("Problem 1:Joint angles for Joints 1, 3, 4 with PD control, Kv = 50");
legend("P1 Q1", "P1 Q3", "P1 Q4", "P2 Q1", "P2 Q3", "P2 Q4");
hold off;

% Problem 3 graph
figure();
plot(ctl2data(:,2),"--r", "LineWidth", 2.0);
hold on;
plot(ctl2data(:,3),"--b", "LineWidth", 2.0);
plot(ctl2data(:, 4),"--g", "LineWidth", 2.0);
hold on; 
plot(ctl3data(:,2),"-r", "LineWidth", 2.0);
hold on;
plot(ctl3data(:,3),"-b","LineWidth", 2.0);
plot(ctl3data(:, 4),"-g","LineWidth", 2.0);
xlabel("SimulationStep" );
ylabel("Joint Angle");
title("Problem 2: Joint angles for Joints 1, 3, 4 with PD control, Kv = 50");
legend("P2 Q1", "P2 Q3", "P2 Q4", "P3 Q1", "P3 Q3", "P3 Q4");
hold off; 

% Problem 4 graph
figure();
plot(ctl3data(:,2),"--r", "LineWidth", 2.0);
hold on;
plot(ctl3data(:,3),"--b", "LineWidth", 2.0);
plot(ctl3data(:, 4),"--g", "LineWidth", 2.0);
plot(ctl4data(:,2),"-r", "LineWidth", 2.0);
hold on;
plot(ctl4data(:,3),"-b","LineWidth", 2.0);
plot(ctl4data(:, 4),"-g","LineWidth", 2.0);
xlabel("SimulationStep" );
ylabel("Joint Angle");
title("Problem 4:Joint angles for Joints 1, 3, 4 with PD control, Kv = 50");
legend("P3 Q1", "P3 Q3", "P3 Q4", "P4 Q1", "P4 Q3", "P4 Q4");

