% telemetry_matlab.m
%
% Live telemetry viewer + logger for Stage4_FullLaw.ino running with
% TELEMETRY_MODE set to TELEMETRY_MATLAB (see the selector near the top
% of Stage4_FullLaw.ino). In that mode the Teensy sends one plain CSV
% line per control cycle, no header/comment lines:
%
%   t_ms,theta_deg,theta_dot_dps,tau_Nm,tau_cmd_Nm,armed,gain_scale,wheel_omega_lp,wheel_pos,wheel_vel
%
% Usage:
%   1. In Stage4_FullLaw.ino, set TELEMETRY_MODE to TELEMETRY_MATLAB and
%      re-upload.
%   2. Set PORT below to the Teensy's serial port (Windows: check Device
%      Manager -> Ports, e.g. "COM5").
%   3. Run this script. A window opens with a live scrolling plot.
%   4. Close the window when you're done -- the full session is saved to
%      a timestamped .csv file next to this script.

clear; clc;

%% ---- settings ----
PORT     = "COM5";     % <-- set to your Teensy's port
BAUD     = 115200;
NUM_COLS = 10;
WINDOW_S = 10;          % seconds visible in the scrolling plot
OUT_DIR  = fileparts(mfilename('fullpath'));

COL_NAMES = ["t_ms", "theta_deg", "theta_dot_dps", "tau_Nm", "tau_cmd_Nm", ...
             "armed", "gain_scale", "wheel_omega_lp", "wheel_pos", "wheel_vel"];

%% ---- serial port ----
sp = serialport(PORT, BAUD);
configureTerminator(sp, "LF");
flush(sp);

%% ---- figure ----
fig = uifigure("Name", "Stage4 Telemetry (MATLAB mode)");
fig.UserData.stop = false;
fig.CloseRequestFcn = @(src, evt) setStop(src);

gridLayout = uigridlayout(fig, [3 1]);

ax1 = uiaxes(gridLayout); title(ax1, "Tilt");    ylabel(ax1, "deg, deg/s");
ax2 = uiaxes(gridLayout); title(ax2, "Torque");  ylabel(ax2, "N\cdotm");
ax3 = uiaxes(gridLayout); title(ax3, "Wheel");   ylabel(ax3, "rad/s"); xlabel(ax3, "t (s)");

hTheta    = animatedline(ax1, "Color", [0.00 0.45 0.74]);
hThetaDot = animatedline(ax1, "Color", [0.85 0.33 0.10]);
legend(ax1, ["theta (deg)", "theta dot (deg/s)"], "Location", "best");

hTau    = animatedline(ax2, "Color", [0.00 0.45 0.74]);
hTauCmd = animatedline(ax2, "Color", [0.85 0.33 0.10], "LineStyle", "--");
legend(ax2, ["tau", "tau cmd"], "Location", "best");

hWheelLp = animatedline(ax3, "Color", [0.00 0.45 0.74]);
legend(ax3, "wheel omega lp", "Location", "best");

%% ---- log buffer (grows in chunks so long sessions don't reallocate every sample) ----
chunk = 20000;
buf = nan(chunk, NUM_COLS);
n = 0;
t0 = [];

%% ---- main loop ----
while isvalid(fig) && ~fig.UserData.stop
    if sp.NumBytesAvailable == 0
        drawnow limitrate;
        continue;
    end

    line = strtrim(readline(sp));
    if isempty(line)
        continue;
    end

    vals = str2double(strsplit(line, ","));
    if numel(vals) ~= NUM_COLS || any(isnan(vals))
        continue;   % malformed/partial line -- skip it
    end

    n = n + 1;
    if n > size(buf, 1)
        buf = [buf; nan(chunk, NUM_COLS)]; %#ok<AGROW>
    end
    buf(n, :) = vals;

    if isempty(t0)
        t0 = vals(1);
    end
    t_s = (vals(1) - t0) / 1000;

    addpoints(hTheta,    t_s, vals(2));
    addpoints(hThetaDot, t_s, vals(3));
    addpoints(hTau,      t_s, vals(4));
    addpoints(hTauCmd,   t_s, vals(5));
    addpoints(hWheelLp,  t_s, vals(8));

    xlim(ax1, [max(0, t_s - WINDOW_S), max(WINDOW_S, t_s)]);
    xlim(ax2, [max(0, t_s - WINDOW_S), max(WINDOW_S, t_s)]);
    xlim(ax3, [max(0, t_s - WINDOW_S), max(WINDOW_S, t_s)]);

    drawnow limitrate;
end

%% ---- cleanup + save ----
clear sp;

if n > 0
    logData = buf(1:n, :);
    stamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    outFile = fullfile(OUT_DIR, "telemetry_" + stamp + ".csv");
    T = array2table(logData, "VariableNames", cellstr(COL_NAMES));
    writetable(T, outFile);
    fprintf("Saved %d samples to %s\n", n, outFile);
else
    fprintf("No data captured -- nothing saved.\n");
end

if isvalid(fig)
    delete(fig);
end

function setStop(fig)
    fig.UserData.stop = true;
end
