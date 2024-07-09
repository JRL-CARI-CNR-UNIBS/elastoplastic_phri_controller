test_1
close all;
fig_properties

fig = figure("WindowState","maximized");
ax1 = gca;
plot(T, z, "LineWidth",LINE_WIDTH);
ax1.Children(1).LineStyle = '--'; % ?
YLabelString = '$z,\,w$ [m]';
if length(ax1.Children) > 2
    ax1.Children(1).LineStyle = ':';
    ax1.Children(1).Color = 'g';
    ax1.Children(2).LineStyle = '--';
    YLabelString = '$z,\,w,\,r$ [m]';
end
ax1.FontSize = TICK_FONT_SIZE;
ax1.YLim = [min(z,[],'all')-0.01, max(z,[],'all')+0.01];
ax1.YLabel.Interpreter = 'latex';
ax1.YLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax1.YLabel.String = YLabelString;
ax1.XLabel.Interpreter = 'latex';
ax1.XLabel.FontSize = AXIS_LABELS_FONT_SIZE;
ax1.XLabel.String = 'Time [s]';
legend("$z$", "$w$", "$r$");
ax1.Legend.FontSize = 40;
ax1.Legend.Interpreter = 'latex';
% ax1.Legend.Location = 'east';
grid on