function target_speed = planning_layer(state, lead_speed, ...
                                       gap, set_speed)

    desired_gap = 30;
    k_gap       = 0.3;

    if strcmp(state, 'CRUISE')
        target_speed = set_speed;

    elseif strcmp(state, 'FOLLOW')
        gap_error    = gap - desired_gap;
        target_speed = lead_speed + k_gap * gap_error;

    elseif strcmp(state, 'BRAKE')
        target_speed = 0;

    else
        target_speed = 0;
    end

    target_speed = max(0, min(target_speed, set_speed));

end
