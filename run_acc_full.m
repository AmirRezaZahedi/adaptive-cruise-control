clear; clc;

run_scenario('Scenario 1 — Free Cruise', ...
    @(t) 999, ...
    9999, 20);

run_scenario('Scenario 2 — Follow Slower Lead', ...
    @(t) 15, ...
    100, 30);

run_scenario('Scenario 3 — Lead Brakes', ...
    @(t) 15 * (t < 15) + 5 * (t >= 15), ...
    100, 35);

run_scenario('Scenario 4 — Lead Accelerates', ...
    @(t) 15 * (t < 15) + 25 * (t >= 15), ...
    100, 35);


function run_scenario(scenario_name, lead_speed_fn, ...
                      lead_pos_init, t_end)

    dt        = 0.1;
    time      = 0:dt:t_end;
    set_speed = 30;

    ego_speed  = 20;
    ego_pos    = 0;
    lead_pos   = lead_pos_init;

    integral   = 0;
    prev_error = 0;

    n = length(time);
    log_ego    = zeros(1,n);
    log_target = zeros(1,n);
    log_gap    = zeros(1,n);
    log_state  = cell(1,n);

    for i = 1:n
        t = time(i);

        lead_speed = lead_speed_fn(t);

        gap           = lead_pos - ego_pos;
        lead_detected = (gap < 250) && (gap > 0);

        state = decision_layer(ego_speed, lead_speed, ...
                               gap, lead_detected, '');

        target_speed = planning_layer(state, lead_speed, ...
                                      gap, set_speed);

        [output, integral, prev_error] = pid_controller(...
            target_speed, ego_speed, dt, integral, prev_error);

        if output >= 0
            throttle = min(output / 30, 1.0);
            brake    = 0;
        else
            throttle = 0;
            brake    = min(abs(output) / 30, 1.0);
        end

        [ego_speed, ~] = vehicle_model(throttle, brake, ...
                                        ego_speed, dt);

        ego_pos  = ego_pos  + ego_speed  * dt;
        lead_pos = lead_pos + lead_speed * dt;

        log_ego(i)    = ego_speed * 3.6;
        log_target(i) = target_speed * 3.6;
        log_gap(i)    = gap;
        log_state{i}  = state;
    end

    figure('Name', scenario_name);

    subplot(3,1,1);
    plot(time, log_ego,    'b-',  'LineWidth', 1.5); hold on;
    plot(time, log_target, 'r--', 'LineWidth', 1.5);
    yline(set_speed*3.6, 'k:', 'Set Speed');
    xlabel('Time (s)'); ylabel('Speed (km/h)');
    title(['Speed Tracking — ' scenario_name]);
    legend('Ego Speed','Target Speed'); grid on;

    subplot(3,1,2);
    plot(time, log_gap, 'g-', 'LineWidth', 1.5);
    yline(30, 'r--', 'Desired Gap (30m)');
    yline(20, 'k:', 'Critical Gap (20m)');
    xlabel('Time (s)'); ylabel('Gap (m)');
    title('Following Distance'); grid on;

    subplot(3,1,3);
    state_num = zeros(1,n);
    for i = 1:n
        if strcmp(log_state{i},'CRUISE'),     state_num(i) = 1;
        elseif strcmp(log_state{i},'FOLLOW'), state_num(i) = 2;
        else,                                 state_num(i) = 3;
        end
    end
    plot(time, state_num, 'k-', 'LineWidth', 1.5);
    yticks([1 2 3]);
    yticklabels({'CRUISE','FOLLOW','BRAKE'});
    xlabel('Time (s)'); ylabel('State');
    title('Decision State'); grid on;

end
