cd("~/CS225A/apps/cs225a/bin/hw2/Controller Data");

ctl2data= importdata("controller1.txt", ',');

figure();
t = ctl2data(:,1);
plot(t,ctl2data(:,2),"-r", "LineWidth", 2.0);
hold on;
plot(t,ctl2data(:,3),"-b", "LineWidth", 2.0);
plot(t,ctl2data(:, 4),"-g", "LineWidth", 2.0);
hold off;
xlabel("time(s)");
ylabel("position(m)");
title({"P1 - EE Position for identifying inertial properties", "Kv = 32, Xdesired = [0.3, 0.1, 0.5]"},"FontSize", 16);
legend("x", "y", "z");
% 
%% Controller 2
% ctl2data = importdata("controller1.txt", ',');
% figure();
% plot(ctl2data(:,2),"-r", "LineWidth", 2.0);
% hold on;
% plot(ctl2data(:,3),"-b", "LineWidth", 2.0);
% plot(ctl2data(:, 4),"-g", "LineWidth", 2.0);
% hold off;
% xlabel("time(s)");
% ylabel("position(m)");
% title({"P1 - EE Position for Operational Space Control", "Kv = 32, Xdesired = [0.3, 0.1, 0.5]"},"FontSize", 16);
% legend("x", "y", "z");
% 
% figure();
% 
% for i = 5:11
%     plot(ctl2data(:,i), "LineWidth", 2.0);
%     hold on;
% end
% xlabel("time(s)", "FontSize", 16);
% ylabel("joint angle (Rad)", "FontSize", 16);
% title({"P2 A - Joint angle for Operational Space Control No Damping", "Kv = 32, Xdesired = [0.3, 0.1, 0.5]"}, "FontSize", 16);
% 
% legend("q1", "q2", "q3","q4", "q5","q6","q7");
% hold off;
% 

%cd("~/CS225A/apps/cs225a/bin/hw2/Controller Data");
%close all;

%% Controller 2C
% ctl2data = importdata("controller2d.txt", ',');
% figure();
% plot(ctl2data(:,2),"-r", "LineWidth", 2.0);
% hold on;
% plot(ctl2data(:,3),"-b", "LineWidth", 2.0);
% plot(ctl2data(:, 4),"-g", "LineWidth", 2.0);
% hold off;
% xlabel("time(s)");
% ylabel("position(m)");
% title({"P2 D - EE Position for Operational Space Control", "Kv = 32, Xdesired = [0.3, 0.1, 0.5]"},"FontSize", 16);
% legend("x", "y", "z");
% 
% figure();
% 
% for i = 5:11
%     plot(ctl2data(:,i), "LineWidth", 2.0);
%     hold on;
% end
% xlabel("time(s)", "FontSize", 16);
% ylabel("joint angle (Rad)", "FontSize", 16);
% title({"P2 D - Joint angle for Operational Space Control No Damping", "Kv = 32, Xdesired = [0.3, 0.1, 0.5]"}, "FontSize", 16);
% 
% legend("q1", "q2", "q3","q4", "q5","q6","q7");
% hold off;

% %% Controller 2C
% ctl2data = importdata("controller4.txt", ',');
% figure();
% hold on;
% t = ctl2data(:,1);
% plot(t,ctl2data(:,2),"-r", "LineWidth", 2.0);
% 
% plot(t,ctl2data(:,3),"-b", "LineWidth", 2.0);
% plot(t,ctl2data(:, 4),"-g", "LineWidth", 2.0);
% 
% plot(t, 0.3 + 0.1*sin(pi*t),"--r", "LineWidth", 2.0);
% plot(t, 0.1 + 0.1*cos(pi*t),"--b", "LineWidth", 1.0);
% plot(t, 0.5,"--g", "LineWidth", 2.0);
% 
% 
% hold off;
% xlabel("time(s)");
% ylabel("position(m)");
% title({"P4 A iv- EE Position for Operational Space Control", "Kp = 400, Kv = 30, Kpq = 50", "Xdesired = [0.3, 0.1, 0.5]"},"FontSize", 16);
% legend("x", "y", "z");
% 
% figure();
% 
% for i = 5:11
%     plot(t, ctl2data(:,i), "LineWidth", 2.0);
%     hold on;
% end
% xlabel("time(s)", "FontSize", 16);
% ylabel("joint angle (Rad)", "FontSize", 16);
% title({"P4 A iv - Joint angle for Operational Space Control", "Kp = 400, Kv = 30, Kpq = 50", "Xdesired = [0.3, 0.1, 0.5]"}, "FontSize", 16);
% 
% legend("q1", "q2", "q3","q4", "q5","q6","q7");
% hold off;




