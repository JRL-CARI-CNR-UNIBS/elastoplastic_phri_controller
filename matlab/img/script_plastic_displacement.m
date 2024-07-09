test_1
close all;

fig_properties

fig = figure("WindowState","maximized");
ax1 = subplot(2,1,1);
plot(T, z, "LineWidth",LINE_WIDTH);
ax1.Children(1).LineStyle = '--'; % ?
ax1.FontSize = TICK_FONT_SIZE;
ax1.YLim = [ax1.YLim(1)-0.01, ax1.YLim(2)+0.01];
ax1.YLabel.Interpreter = 'latex';
ax1.YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax1.YLabel.String = '$z,\,w$ [m]';
legend("$z$", "$w$", "$r$");
ax1.Legend.FontSize = 40;
ax1.Legend.Interpreter = 'latex';
ax1.Legend.Location = 'east';
grid on

ax2 = subplot(2,1,2);
plot(T, x, "LineWidth",LINE_WIDTH);
ax2.FontSize = TICK_FONT_SIZE;
ax2.YLim = [ax2.YLim(1)-0.25, ax2.YLim(2)+0.25];
ax2.YLabel.Interpreter = 'latex';
ax2.YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax2.YLabel.String = '$x_d$ [m]';
ax2.XLabel.Interpreter = 'latex';
ax2.XLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax2.XLabel.String = 'Time [s]';
grid on