%
% Figure: Experimental-2
%

close all; clear; clc;

fig_properties

ENABLE_SAVE = true;

addpath("../data/prova_3");
tab = readtable("prova_3.csv");

dt = 0.002;

z(1,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_z_data_0_);
z(2,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_z_data_1_);
z(3,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_z_data_2_);

w(1,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_w_data_0_);
w(2,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_w_data_1_);
w(3,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_w_data_2_);

F(1,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_wrench_in_base_wrench_force);
F(2,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_wrench_in_base_wrench_for_1);
F(3,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_wrench_in_base_wrench_for_2);

err(1,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_cart_err_data_0_);
err(2,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_cart_err_data_1_);
err(3,:) = rmmissing(tab.x_ur10e_hw_elastoplastic_controller_cart_err_data_2_);

t = 0:dt:dt*length(z(1,:));

ax = [];

fig = figure("WindowState","maximized");
ax = [ax subplot(3,1,1)];
hold on
for idx=1:3
    plot(t, F(idx,:), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
end
ax(1).FontSize = TICK_FONT_SIZE;
ax(1).XLim = [28,49];
ax(1).XTickLabel = ["0","2","4","6","8","10","12","14","16","18","20"];
ax(1).YLabel.String = ["$F_h$ [N]"];
ax(1).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(1).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(1).YLabel.Interpreter = 'latex';
legend("$F_{h,x}$","$F_{h,y}$","$F_{h,z}$", 'interpreter', 'latex')
grid on
box on
hold off

ax = [ax subplot(3,1,2)];
hold on
for idx=1:3
    plot(t(1:end-1), z(idx,:), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
end
ax(2).FontSize = TICK_FONT_SIZE;
ax(2).XLim = [28,49];
ax(2).XTickLabel = ["0","2","4","6","8","10","12","14","16","18","20"];
ax(2).YLim = [-0.3,0.4];
ax(2).YLabel.String = ["$z$ [m]"];
ax(2).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(2).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(2).YLabel.Interpreter = 'latex';
legend("$z_{x}$","$z_{y}$","$z_{z}$", 'interpreter', 'latex')
grid on
box on
hold off

ax = [ax subplot(3,1,3)];
hold on
for idx=1:3
    plot(t(1:end-1), err(idx,:), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
end
ax(3).FontSize = TICK_FONT_SIZE;
ax(3).XLim = [28,49];
ax(3).XTickLabel = ["0","2","4","6","8","10","12","14","16","18","20","22"];
ax(3).YLim = [-0.3,0.3];
ax(3).YLabel.String = ["$x_d$ [m]"];
ax(3).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(3).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(3).YLabel.Interpreter = 'latex';
ax(3).XLabel.String = "Time [s]";
ax(3).XLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(3).XLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(3).XLabel.Interpreter = 'latex';

legend("$err_{x}$","$err_{y}$","$err_{z}$", 'interpreter', 'latex')
grid on
box on
hold off

NAME_FILE = "experimental-2";

if ENABLE_SAVE
    check_input = input("Save Image? ",'s');
    if ~strcmp(check_input,'s')
        disp("Operation cancelled")
        return
    end
    fprintf("Saving %s.fig and %s.eps ...", NAME_FILE, NAME_FILE)
    savefig(fig, strcat(NAME_FILE,".fig"))
    exportgraphics(fig, strcat(NAME_FILE,".eps"))
    fprintf(" Done!\n")
end