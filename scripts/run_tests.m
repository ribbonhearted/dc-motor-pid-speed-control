%% run_tests.m - Model-in-the-Loop (MIL) tests for DC motor speed control
%
% Runs the Simulink model with P, PI and PID controllers, measures the
% response, checks it against the requirements and saves the results.
%
% Requirements (see docs/requirements.md):
%   REQ-01  Settling time (2% band) < 2 s for a 1 rad/s speed step
%   REQ-02  Overshoot < 5%
%   REQ-03  Steady-state error < 1%
%   REQ-04  After a load torque step, speed returns within 2% of the
%           reference in < 1 s
%
% Before running:
%   - Current Folder = project root (the folder that contains models/,
%     scripts/, images/, results/)
%   - models/ and scripts/ are on the MATLAB path
%   - PID block gains are set to Kp, Ki, Kd
%   - Load torque Step block uses t_load and T_load_step (from params.m)
%   - To Workspace blocks named "speed" and "voltage", format Timeseries

clear; clc; close all;
params;                                   % motor parameters + test conditions
model = 'dc_motor_speed_control';

labels = ["FAIL" "PASS"];
pf = @(c) labels(c + 1);                  % true/false -> "PASS"/"FAIL"

if ~isfolder('images'),  mkdir('images');  end
if ~isfolder('results'), mkdir('results'); end

%% Test cases
%        name    Kp    Ki   Kd
cases = {'P',   100,    0,   0;
         'PI',  100,  200,   0;
         'PID', 100,  200,  10};

results = table();

%% Run each test case
for k = 1:size(cases, 1)
    name = cases{k,1};
    Kp = cases{k,2};  Ki = cases{k,3};  Kd = cases{k,4};

    out = sim(model);                     % run the Simulink model
    t = out.speed.Time;    y = squeeze(out.speed.Data);
    tu = out.voltage.Time; u = squeeze(out.voltage.Data);

    % --- Step-response metrics (only before the load step) ---
    before = t < t_load;
    info   = stepinfo(y(before), t(before), 'SettlingTimeThreshold', 0.02);
    y_ss   = y(find(before, 1, 'last'));
    ss_err = abs(speed_ref - y_ss) / speed_ref * 100;      % [%]

    % --- Disturbance metrics (REQ-04) ---
    after   = t >= t_load;
    max_dip = (speed_ref - min(y(after))) / speed_ref * 100; % [%]
    outside = after & abs(y - speed_ref) > 0.02 * speed_ref;
    if any(outside)
        recovery = t(find(outside, 1, 'last')) - t_load;     % [s]
    else
        recovery = 0;
    end

    % --- Requirement checks ---
    req = [info.SettlingTime < 2, ...     % REQ-01
           info.Overshoot    < 5, ...     % REQ-02
           ss_err            < 1, ...     % REQ-03
           recovery          < 1];        % REQ-04

    row = table(string(name), info.RiseTime, info.Overshoot, info.SettlingTime, ...
                ss_err, max_dip, recovery, ...
                pf(req(1)), pf(req(2)), pf(req(3)), pf(req(4)), max(abs(u)), ...
        'VariableNames', {'Controller', 'RiseTime_s', 'Overshoot_pct', ...
                          'SettlingTime_s', 'SSError_pct', 'MaxDip_pct', ...
                          'Recovery_s', 'REQ01', 'REQ02', 'REQ03', 'REQ04', ...
                          'MaxVoltage_V'});
    results = [results; row]; %#ok<AGROW>

    % --- Plots: speed and voltage ---
    figure('Name', [name ' controller']);
    subplot(2,1,1);
    plot(t, y, 'LineWidth', 1.5); hold on;
    yline(speed_ref, '--', 'Reference');
    xline(t_load, ':', 'Load step');
    title([name ' controller - speed response']);
    ylabel('Speed [rad/s]'); grid on;

    subplot(2,1,2);
    plot(tu, u, 'LineWidth', 1.5);
    xline(t_load, ':');
    title('Controller output (motor voltage)');
    xlabel('Time [s]'); ylabel('Voltage [V]'); grid on;

    saveas(gcf, fullfile('images', [lower(name) '_test.png']));
end

%% Show and save results
disp(results);
writetable(results, fullfile('results', 'test_results.csv'));

n_pass = sum(all(results{:, {'REQ01','REQ02','REQ03','REQ04'}} == "PASS", 2));
fprintf('\n%d of %d controllers pass all requirements.\n', n_pass, height(results));