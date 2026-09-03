function state = decision_layer(ego_speed, lead_speed, ...
gap, lead_detected, prev_state)


safe_distance     = 80;
critical_distance = 20;
closing_threshold = 5;

closing_speed = ego_speed - lead_speed;

if ~lead_detected
    state = 'CRUISE';

elseif gap < critical_distance || closing_speed > closing_threshold
    state = 'BRAKE';

elseif gap < safe_distance
    state = 'FOLLOW';

else
    state = 'CRUISE';
end


end
