clear; close all; clc;

models.LUGRE = 1;
models.ALPHA_LUGRE = 2;
models.ELASTOPLASTIC_NO_R = 3;
models.ELASTOPLASTIC = 4;

MODEL = models.ELASTOPLASTIC;

ENABLE_INPUT_TRAJECTORY = 0;

amp = 100; % Input force amplitude
noise_amp = 0;

dt = 1e-3;
Tend = 35;

s0 = 1000;
s1 = 100;
s2 = 50;
fss = 100;

m = 1;

z_ss = fss/s0;
z_ba = 0.3*z_ss;

kw = 50;

reset_window_size = 1000;
reset_ths = 1;

fprintf("z steady-state = %f\n", fss/s0);
fprintf("z_ss = %f\n", fss/s0);
fprintf("z_ba = %f\n", z_ba);
fprintf("s0 = %f\n",  s0);
fprintf("s1 = %f\n",  s1);
fprintf("s2 = %f\n",  s2);
fprintf("fss = %f\n", fss);

T = 0:dt:Tend;

z =   zeros(length(T), 1);
x =   zeros(length(T), 1);
xp =  zeros(length(T), 1);
xpp = zeros(length(T), 1);

fr = zeros(length(T), 1);

%% Choose model
enable_reset = 0;
z_size = 1;
switch MODEL
    case models.LUGRE
        elastoplastic_model = @(z,v) lugre(z, v, s0, s1, s2, fss);
    case models.ALPHA_LUGRE
        elastoplastic_model = @(z,v) alpha_lugre(z, v, s0, s1, s2, fss, z_ba, z_ss);
    case models.ELASTOPLASTIC_NO_R
        elastoplastic_model = @(z,v) elastoplastic_no_r(z, v, s0, s1, s2, fss, z_ba, z_ss, kw);
        z_size = 2;
    case models.ELASTOPLASTIC
        elastoplastic_model = @(z,v) elastoplastic(z, v, s0, s1, s2, fss, z_ba, z_ss, kw);
        enable_reset = 1;
        z_size = 3;
    otherwise
        disp("MODELLO NON VALIDO")
        return
end

input_trajectory = zeros(length(T), 2);
if ENABLE_INPUT_TRAJECTORY
    fprintf("Input Trajectory: ENABLED\n");
    input_trajectory(:, 1) = T;
    input_trajectory(1:end-1, 2) = diff(input_trajectory(:, 1))/dt;
    input_trajectory(end, 2) = input_trajectory(end-1, 2);  
end


%% Constant Velocity
for idx = 2:length(T)
    [zp, ~] = lugre(z(idx-1), 0.005, s0, s1, s2, fss);
    z(idx) = z(idx-1) + zp * dt;
end

% figure
% plot(T, z,"LineWidth",1.5);
% title("constant velocity")
% grid on

%% Evolution
z = zeros(length(T), z_size);
u = zeros(length(T), 1);
zp_ = zeros(length(T), z_size);

reset_window = [];

for idx = 1:length(T)-1
    u(idx) = amp*input_force(T(idx)) + noise_amp * randn();
    [zp, fr(idx)] = elastoplastic_model(z(idx, :), xp(idx) - input_trajectory(idx, 2));
    z(idx+1, :) = z(idx, :) + zp * dt;
    xpp(idx+1) = (u(idx) - fr(idx)) / m;
    xp(idx+1)  = xp(idx) + xpp(idx) * dt;
    x(idx+1)   = x(idx) + xp(idx) * dt;
    zp_(idx+1, :) = zp;
    if(enable_reset && alpha(norm([z(idx, 1) z(idx, 3)]), z_ba, z_ss) > 0)
        [z(idx+1, :), reset_window] = reset_condition(dt, z(idx+1, :), reset_window, xp(idx), u(idx), reset_window_size, reset_ths);
    end
end

figure
subplot(3,1,1)
hold on
plot(T, u,"LineWidth",1.5);
plot(T, fr,"LineWidth",1.5);
legend("Input force", "Friction force")
hold off
grid on
subplot(6,1,3)
hold on
plot(T, xp,"--","LineWidth",1.5)
plot(T, xpp,"LineWidth",1.5)
hold off
legend("xp", "xpp")
grid on
subplot(6,1,4)
plot(T, x,"-.","LineWidth",1.5)
legend("x")
grid on
subplot(3,2,5)
plot(T, zp_, "LineWidth",1.5);
legend("zp", "wp", "rp")
grid on
subplot(3,2,6)
plot(T, z, "LineWidth",1.5);
legend("z", "w", "r")
grid on

%% INPUT
function out = input_force(t)
    if t < 0
        out = 0;
    elseif t < 5
                    out = 0.2;
    elseif t < 10
                    out = 0;
    elseif t < 15
                    out = 1;
    elseif t < 20
                    out = -1;
    elseif t < 25
                    out = 0;
    else
                    out = 0;
    end
%     if t < 10
%         out = 0;
%     else
%         out = t-10;
%     end
end

%% ELASTOPLASTIC MODELS
% Lugre
function [zp, fr] = lugre(z,v,s0,s1,s2,fss)
    zp = v - abs(v) * s0/fss * z;
    fr = s0*z + s1*zp + s2*v;
end

function a = alpha(z, z_ba, z_ss)
    z = abs(z);
    if(z < z_ba)
        a = 0;
    elseif(z >= z_ba && z < z_ss)
        a = 0.5*sin(pi*(z-(z_ss+z_ba)/2)/(z_ss-z_ba)) + 0.5;
%         a = 0;
    else
        a = 1;
    end
end

function a = alpha_ext(v, z, z_ba, z_ss)
    if sign(v) == sign(z)
        a = alpha(z, z_ba, z_ss);
    else
        a = 0;
    end
end

% Lugre with Alpha
function [zp, fr] = alpha_lugre(z,v,s0,s1,s2,fss,z_ba,z_ss)
    zp = v - alpha_ext(v,z,z_ba,z_ss) * abs(v) * s0/fss * z;
    fr = s0*z + s1*zp + s2*v;
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

% Elastoplastic without r
function [zp_ext, fr] = elastoplastic_no_r(z_ext,v,s0,s1,s2,fss,z_ba,z_ss,kw)
    z = z_ext(1);
    w = z_ext(2);
    A = alpha(z,z_ba,z_ss);
    zp = v - A * abs(v) * s0/fss * z;
    wp = A * kw * (z - w);
    fr = s0*(z-w) + s1*zp + s2*v;
    zp_ext = [zp, wp];
end

% Elastoplastic
function [zp_ext, fr] = elastoplastic(z_ext,v,s0,s1,s2,fss,z_ba,z_ss,kw)
    z = z_ext(1);
    w = z_ext(2);
    r = z_ext(3);
    A = alpha(norm([z, r]),z_ba,z_ss);
    zp = v - A * abs(v) * s0/fss * z;
    rp = d_alpha(max([z r]), z_ba, z_ss);
    wp = A * kw * (z - w);
    fr = s0*(z-w) + s1*zp + s2*v;
    zp_ext = [zp, wp, rp];
end

function [out, reset_window] = reset_condition(dt, z_ext, reset_window, v, fh, reset_window_size, reset_ths)
    z = z_ext(1);
    reset_window = [reset_window fh * v];
    if(length(reset_window) > reset_window_size)
        reset_window = reset_window(2:end);
    end
    if(length(reset_window) >= reset_window_size && sum(reset_window)*dt < reset_ths)
        out = zeros(1,length(z));
        reset_window = [];
    else
        out = z_ext;
    end
end