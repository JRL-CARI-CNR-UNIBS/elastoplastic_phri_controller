%
% Figure: experimental-1
%

close all; clear; clc;

fig_properties

NAME_FILE = "video-experimental-1";
ENABLE_SAVE = false;

addpath("../data/prova_a3");
tab = readtable("prova_a3.csv"); 

t = rmmissing(tab.x__time);
t = (t - t(1))';

T_START = 35;
T_END = 48; %65;

z(1,:) = tab.x_ur10e_hw_elastoplastic_controller_z_data_0_(t > T_START & t < T_END);
z(2,:) = tab.x_ur10e_hw_elastoplastic_controller_z_data_1_(t > T_START & t < T_END);
z(3,:) = tab.x_ur10e_hw_elastoplastic_controller_z_data_2_(t > T_START & t < T_END);

w(1,:) = tab.x_ur10e_hw_elastoplastic_controller_w_data_0_(t > T_START & t < T_END);
w(2,:) = tab.x_ur10e_hw_elastoplastic_controller_w_data_1_(t > T_START & t < T_END);
w(3,:) = tab.x_ur10e_hw_elastoplastic_controller_w_data_2_(t > T_START & t < T_END);

F(1,:) = tab.x_ur10e_hw_elastoplastic_controller_wrench_in_base_wrench_force(t > T_START & t < T_END);
F(2,:) = tab.x_ur10e_hw_elastoplastic_controller_wrench_in_base_wrench_for_1(t > T_START & t < T_END);
F(3,:) = tab.x_ur10e_hw_elastoplastic_controller_wrench_in_base_wrench_for_2(t > T_START & t < T_END);

Fr(1,:) = tab.x_ur10e_hw_elastoplastic_controller_Fr_in_base_wrench_force_x(t > T_START & t < T_END);
Fr(2,:) = tab.x_ur10e_hw_elastoplastic_controller_Fr_in_base_wrench_force_y(t > T_START & t < T_END);
Fr(3,:) = tab.x_ur10e_hw_elastoplastic_controller_Fr_in_base_wrench_force_z(t > T_START & t < T_END);

err(1,:) = tab.x_ur10e_hw_elastoplastic_controller_pose_of_t_in_b_pose_positio(t > T_START & t < T_END);
err(2,:) = tab.x_ur10e_hw_elastoplastic_controller_pose_of_t_in_b_pose_posit_1(t > T_START & t < T_END);
err(3,:) = tab.x_ur10e_hw_elastoplastic_controller_pose_of_t_in_b_pose_posit_2(t > T_START & t < T_END);

t = t(t > T_START & t < T_END);
t = t - t(1);

ax = [];

idx = 1;

fig = figure("WindowState","maximized","Color",	"#D2D2D2");
ax = [ax subplot(3,1,1)];
hold on
al1 = animatedline(ax(1), "LineStyle",LINE_STYLES(1),"LineWidth",LINE_WIDTH, "Color", LINE_COLORS(1));
al11 = animatedline(ax(1), "LineStyle",LINE_STYLES(2),"LineWidth",LINE_WIDTH, "Color", LINE_COLORS(2));
% plot(t(~isnan(F(1,:))), F(idx,~isnan(F(1,:))), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
% plot(t(~isnan(Fr(1,:))), Fr(idx,~isnan(Fr(1,:))), "LineStyle",LINE_STYLES(mod(idx+1,length(LINE_STYLES))),"LineWidth",LINE_WIDTH);
xlim([0 T_END-T_START])
ax(1).FontSize = TICK_FONT_SIZE;
ax(1).YLim = [min(F,[],'all')-5, max(F,[],'all')+5];
ax(1).YLabel.String = ["$F_h, F_r$ [N]"];
ax(1).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(1).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(1).YLabel.Interpreter = 'latex';
legend("$F_{h,x}$","$F_{r,x}$", 'interpreter', 'latex')
grid on
box on
hold off

ax = [ax subplot(3,1,2)];
hold on
al2 = animatedline(ax(2), "LineStyle",LINE_STYLES(1),"LineWidth",LINE_WIDTH, "Color", LINE_COLORS(1));
al22 = animatedline(ax(2), "LineStyle",LINE_STYLES(2),"LineWidth",LINE_WIDTH, "Color", LINE_COLORS(2));
% plot(t(~isnan(z(1,:))), z(idx,~isnan(z(1,:))), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
% plot(t(~isnan(w(1,:))), w(idx,~isnan(w(1,:))), "LineStyle",LINE_STYLES(mod(idx+1,length(LINE_STYLES))),"LineWidth",LINE_WIDTH);
xlim([0 T_END-T_START])
ax(2).FontSize = TICK_FONT_SIZE;
% ax(2).XLim = [28,49];
ax(2).YLim = [min(z,[],'all')-0.1, max(z,[],'all')+0.1];
ax(2).YLabel.String = ["$z, w$ [m]"];
ax(2).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(2).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(2).YLabel.Interpreter = 'latex';
legend("$z_{x}$","$w_{x}$", 'interpreter', 'latex')
grid on
box on
hold off

ax = [ax subplot(3,1,3)];
hold on
al3 = animatedline(ax(3), "LineStyle",LINE_STYLES(1),"LineWidth",LINE_WIDTH, "Color", LINE_COLORS(1));
xlim([0 T_END-T_START])
% plot(t(~isnan(err(1,:))), err(idx,~isnan(err(1,:))), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH);
ax(3).FontSize = TICK_FONT_SIZE;
% ax(3).XLim = [28,49];
% ax(3).YLim = [-0.3,0.3];
ax(3).YLim = [min(err,[],'all')-0.1, max(err,[],'all')+0.1];
ax(3).YLabel.String = ["$x$ [m]"];
ax(3).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(3).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(3).YLabel.Interpreter = 'latex';
ax(3).XLabel.String = "Time [s]";
ax(3).XLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(3).XLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(3).XLabel.Interpreter = 'latex';

legend("$x_{x}$", 'interpreter', 'latex')
grid on
box on
hold off

ff1 = find(~isnan(F(1,:)));
ff11 = find(~isnan(Fr(1,:)));
ff2 = find(~isnan(z(1,:)));
ff22 = find(~isnan(w(1,:)));
ff3 = find(~isnan(err(1,:)));

myVideo = VideoWriter('exp_1_animation.avi'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

loop_rate = 50;
idx = 1;
for tdx = 1:length(t(ff1))
        addpoints(al1, t(ff1(tdx)), F(idx,ff1(tdx)));
        addpoints(al11, t(ff11(tdx)), Fr(idx,ff11(tdx)));
        addpoints(al2, t(ff2(tdx)), z(idx,ff2(tdx)));
        addpoints(al22, t(ff22(tdx)), w(idx,ff22(tdx)));
        addpoints(al3, t(ff3(tdx)), err(idx,ff3(tdx)));
    if mod(tdx,loop_rate) == 0
        drawnow
        frame = getframe(gcf); %get frame
        writeVideo(myVideo, frame);
    end
end

close(myVideo)

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