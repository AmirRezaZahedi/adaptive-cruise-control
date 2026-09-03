function [output, integral, prev_error] = pid_controller(...
            target, current, dt, integral, prev_error)

    Kp = 0.5;
    Ki = 0.1;
    Kd = 0.05;

    error = target - current;

    integral = integral + error * dt;

    derivative = (error - prev_error) / dt;

    output = Kp * error + Ki * integral + Kd * derivative;

    prev_error = error;

end
