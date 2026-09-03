function [v_new, a] = vehicle_model(throttle, brake, v, dt)

    mass       = 1500;
    max_force  = 3000;
    max_brake  = 5000;
    drag_coeff = 50;

    F_throttle = throttle * max_force;
    F_brake    = brake    * max_brake;
    F_drag     = drag_coeff * v;

    F_net = F_throttle - F_brake - F_drag;

    a = F_net / mass;

    v_new = v + a * dt;

    if v_new < 0
        v_new = 0;
    end

end
