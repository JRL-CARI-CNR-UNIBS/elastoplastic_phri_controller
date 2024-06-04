%
% Figure: experimental-1
%

close all; clear; clc;

fig_properties

NAME_FILE = "experimental-1";
ENABLE_SAVE = false  ;

addpath("../data/prova_5");
tab = readtable("prova_5.csv"); 

t = rmmissing(tab.x__time);
t = (t - t(1))';

T_START = 41;
T_END = 53 ; %65;

z(1,:) = tab.x_ur10e_hw_elastoplastic_controller_z_data_0_(t > T_START & t < T_END);
z(2,:) = tab.x_ur10e_hw_elastoplastic_controller_z_data_1_(t > T_START & t < T_END);
z(3,:) = tab.x_ur10e_hw_elastoplastic_controller_z_data_2_(t > T_START & t < T_END);

w(1,:) = tab.x_ur10e_hw_elastoplastic_controller_w_data_0_(t > T_START & t < T_END);
w(2,:) = tab.x_ur10e_hw_elastoplastic_controller_w_data_1_(t > T_START & t < T_END);
w(3,:) = tab.x_ur10e_hw_elastoplastic_controller_w_data_2_(t > T_START & t < T_END);

F(1,:) = tab.x_ur10e_hw_elastoplastic_controller_wrench_in_base_wrench_force(t > T_START & t < T_END);
F(2,:) = tab.x_ur10e_hw_elastoplastic_controller_wrench_in_base_wrench_for_1(t > T_START & t < T_END);
F(3,:) = tab.x_ur10e_hw_elastoplastic_controller_wrench_in_base_wrench_for_2(t > T_START & t < T_END);

err(1,:) = tab.x_ur10e_hw_elastoplastic_controller_pose_of_t_in_b_pose_positio(t > T_START & t < T_END);
err(2,:) = tab.x_ur10e_hw_elastoplastic_controller_pose_of_t_in_b_pose_posit_1(t > T_START & t < T_END);
err(3,:) = tab.x_ur10e_hw_elastoplastic_controller_pose_of_t_in_b_pose_posit_2(t > T_START & t < T_END);

t = t(t > T_START & t < T_END);
t = t - t(1);

ax = [];

% plot(t(~isnan(F(1,:))), F(:,~isnan(F(1,:))))

fig = figure("WindowState","maximized");
ax = [ax subplot(3,1,1)];
hold on
for idx=1:3
    plot(t(~isnan(F(1,:))), F(idx,~isnan(F(1,:))), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
end
ax(1).FontSize = TICK_FONT_SIZE;
% ax(1).XLim = [28,49];
ax(1).YLim = [ax(1).YLim(1)-5, ax(1).YLim(2)+5];
ax(1).YLabel.String = ["$F_h$ [N]"];
ax(1).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(1).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(1).YLabel.Interpreter = 'latex';
legend("$F_{h,x}$","$F_{h,y}$","$F_{h,z}$", 'interpreter', 'latex')
grid on
hold off

ax = [ax subplot(3,1,2)];
hold on
for idx=1:3
    plot(t(~isnan(z(1,:))), z(idx,~isnan(z(1,:))), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
end
% for idx=1:3
%     plot(t(~isnan(w(1,:))), w(idx,~isnan(w(1,:))), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
% end
ax(2).FontSize = TICK_FONT_SIZE;
% ax(2).XLim = [28,49];
ax(2).YLim = [ax(2).YLim(1)-0.1, ax(2).YLim(2)+0.1];
ax(2).YLabel.String = ["$z$ [m]"];
ax(2).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(2).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(2).YLabel.Interpreter = 'latex';
legend("$z_{x}$","$z_{y}$","$z_{z}$", 'interpreter', 'latex')
grid on
hold off

ax = [ax subplot(3,1,3)];
hold on
for idx=1:3
    plot(t(~isnan(err(1,:))), err(idx,~isnan(err(1,:))), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
end
ax(3).FontSize = TICK_FONT_SIZE;
% ax(3).XLim = [28,49];
% ax(3).YLim = [-0.3,0.3];
ax(3).YLim = [ax(3).YLim(1)-0.1, ax(3).YLim(2)+0.1];
ax(3).YLabel.String = ["$x$ [m]"];
ax(3).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(3).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(3).YLabel.Interpreter = 'latex';
ax(3).XLabel.String = "Time [s]";
ax(3).XLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(3).XLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(3).XLabel.Interpreter = 'latex';

legend("$x_{x}$","$x_{y}$","$x_{z}$", 'interpreter', 'latex')
grid on
hold off

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