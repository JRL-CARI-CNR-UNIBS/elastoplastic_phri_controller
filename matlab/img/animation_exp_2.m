%
% Figure: experimental-1
%

close all; clear; clc;

fig_properties

NAME_FILE = "video-experimental-2";
ENABLE_SAVE = false; 

addpath("../data/prova_ai3");
tab = readtable("prova_ai3.csv"); 

t = rmmissing(tab.x__time);
t = (t - t(1))';

T_START = 0;
T_END = 20; %65;

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

% err(1,:) = tab.x_ur10e_hw_elastoplastic_controller_cart_err_data_1_(t > T_START & t < T_END);
% err(2,:) = tab.x_ur10e_hw_elastoplastic_controller_cart_err_data_2_(t > T_START & t < T_END);
% err(3,:) = tab.x_ur10e_hw_elastoplastic_controller_cart_err_data_3_(t > T_START & t < T_END);

err(1,:) = tab.x_ur10e_hw_elastoplastic_controller_pose_of_t_in_b_pose_positio(t > T_START & t < T_END) - tab.x_ur10e_hw_elastoplastic_controller_target_of_t_in_b_pose_posit(t > T_START & t < T_END);
err(2,:) = tab.x_ur10e_hw_elastoplastic_controller_pose_of_t_in_b_pose_posit_1(t > T_START & t < T_END) - tab.x_ur10e_hw_elastoplastic_controller_target_of_t_in_b_pose_pos_1(t > T_START & t < T_END);
err(3,:) = tab.x_ur10e_hw_elastoplastic_controller_pose_of_t_in_b_pose_posit_2(t > T_START & t < T_END) - tab.x_ur10e_hw_elastoplastic_controller_target_of_t_in_b_pose_pos_2(t > T_START & t < T_END);

t = t(t > T_START & t < T_END);
t = t - t(1);

al1 = gobjects(3,1);
al2 = gobjects(3,1);
al3 = gobjects(3,1);


ax = [];

% plot(t(~isnan(F(1,:))), F(:,~isnan(F(1,:))))

fig = figure("WindowState","maximized","Color",	"#D2D2D2");
ax = [ax subplot(3,1,1)];
for idx=1:3
    al1(idx) = animatedline(ax(1), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH, "Color", LINE_COLORS(idx));
end
hold on
xlim([0 T_END-T_START])
% plot(t(~isnan(Fr(1,:))), Fr(idx,~isnan(Fr(1,:))), "LineStyle",LINE_STYLES(mod(idx+1,length(LINE_STYLES))),"LineWidth",LINE_WIDTH);
ax(1).FontSize = TICK_FONT_SIZE;
ax(1).YLim = [min(F,[],'all')-5, max(F,[],'all')+5];
ax(1).YLabel.String = ["$F_h$ [N]"];
ax(1).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(1).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(1).YLabel.Interpreter = 'latex';
legend("$F_{h,x}$","$F_{h,y}$","$F_{h,z}$", 'interpreter', 'latex')
grid on
box on
hold off

ax = [ax subplot(3,1,2)];
for idx=1:3
    al2(idx) = animatedline(ax(2), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH, "Color", LINE_COLORS(idx));
end
hold on
xlim([0 T_END-T_START])
% plot(t(~isnan(w(1,:))), w(idx,~isnan(w(1,:))), "LineStyle",LINE_STYLES(mod(idx+1,length(LINE_STYLES))),"LineWidth",LINE_WIDTH);
ax(2).FontSize = TICK_FONT_SIZE;
ax(2).YLim = [min(z,[],'all')-0.1, max(z,[],'all')+0.1];
ax(2).YLabel.String = ["$z$ [m]"];
ax(2).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(2).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(2).YLabel.Interpreter = 'latex';
legend("$z_{x}$","$z_{y}$","$z_{z}$", 'interpreter', 'latex')
grid on
box on
hold off

ax = [ax subplot(3,1,3)];
for idx=1:3
    al3(idx) = animatedline(ax(3), "LineStyle",LINE_STYLES(idx),"LineWidth",LINE_WIDTH, "Color", LINE_COLORS(idx));
end
hold on
xlim([0 T_END-T_START])
ax(3).FontSize = TICK_FONT_SIZE;
% ax(3).XLim = [28,49];
% ax(3).YLim = [-0.3,0.3];
ax(3).YLim = [min(err,[],'all')-0.1, max(err,[],'all')+0.1];
ax(3).YLabel.String = ["$x_d$ [m]"];
ax(3).YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(3).YLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(3).YLabel.Interpreter = 'latex';
ax(3).XLabel.String = "Time [s]";
ax(3).XLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax(3).XLabel.FontWeight = LABEL_FONT_WEIGHT;
ax(3).XLabel.Interpreter = 'latex';
legend("$x_{d,x}$","$x_{d,y}$","$x_{d,z}$", 'interpreter', 'latex')
grid on
box on
hold off

ff1 = find(~isnan(F(1,:)));
ff2 = find(~isnan(z(1,:)));
ff3 = find(~isnan(err(1,:)));

pause 

myVideo = VideoWriter('exp_2_animation.avi'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

loop_rate = 50;

for tdx = 1:length(t(ff1))
    for idx = 1:3
        addpoints(al1(idx), t(ff1(tdx)), F(idx,ff1(tdx)));
    end
    for idx = 1:3
        addpoints(al2(idx), t(ff2(tdx)), z(idx,ff2(tdx)));
    end
    for idx = 1:3
        addpoints(al3(idx), t(ff3(tdx)), err(idx,ff3(tdx)));
    end
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