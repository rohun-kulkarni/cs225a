%% Plots for Controller 3

load("ctldataAll3.mat");

dims = size(ctldataAll3);
num_trials = mod(dims(2),4) + 1;
ctldata = importdata("controller3.txt", ',');
size_trial = size(ctldata);
hold off;

ctldataAll3 = {ctldataAll3, ctldata};

% figure();
% plot(ctldataAll{end}(:,2), "LineWidth", 2.0);
% hold on;
% plot(ctldataAll{end}(:,3), "LineWidth", 2.0);
% plot(ctldataAll1{end}(:, 4), "LineWidth", 2.0);

plot(ctldataAll3{end}(:,2), "LineWidth", 2.0);
hold on;
plot(ctldataAll3{end}(:,3), "LineWidth", 2.0);
plot(ctldataAll3{end}(:, 4), "LineWidth", 2.0);
xlabel("SimulationStep" );
ylabel("Joint Angle");
title("Joint angles for Joints 1, 3, 4 with PD control");
legend("Q1", "Q3", "Q4");
save("ctldataAll2.mat");

hold off;
