%
% Figure: alpha and alpha dot
%

clear; clc; close all;

fig_properties

ENABLE_SAVE = true;

z_ba = 0.5;
z_ss = 1.5;

A = @(z) alpha(z, z_ba, z_ss);
dA = @(z) d_alpha(z, z_ba, z_ss);

% T = 0:0.002:2;
% v = zeros(1, length(T));
% for idx = 1:length(T)
%     v(idx) = d_alpha(T(idx), z_ba, z_ss) * 0.002;
% end


fig = figure("WindowState","maximized");
hold on
fplot(A, "LineStyle",LINE_STYLES(1),"LineWidth",LINE_WIDTH)
fplot(dA, "LineStyle",LINE_STYLES(2),"LineWidth",LINE_WIDTH)
% plot(T,cumsum(v));
ax = gca;
ax.FontSize = TICK_FONT_SIZE;
xlim([z_ba-0.5 z_ss+0.5])
ylim([-.2 1.8])
legend('$\alpha(z)$','$\dot{\alpha}(z)$','interpreter','latex')
grid on
hold off

NAME_FILE = "alpha_and_dot_alpha";

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

function a = alpha(z, z_ba, z_ss)
    z = abs(z);
    if(z < z_ba)
        a = 0;
    elseif(z >= z_ba && z < z_ss)
        a = 0.5*sin(pi*(z-(z_ss+z_ba)/2)/(z_ss-z_ba)) + 0.5;
    else
        a = 1;
    end
end

function out = d_alpha(z, z_ba, z_ss)
    z = abs(z);
    if(z < z_ba)
        out = 0;
    elseif(z >= z_ba && z < z_ss)
        out = 0.5*cos(pi*(z-(z_ss+z_ba)/2)/(z_ss-z_ba))*pi/(z_ss-z_ba);
    else
        out = 0;
    end
end