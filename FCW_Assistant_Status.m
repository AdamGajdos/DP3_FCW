function fcw_status = FCW_Assistant_Status(is_fcw_on, velocity, gear)
    %% Serves to determine the state in which FCW assistant is at the moment
    % * -1 for STATUS OFF
    % * 0 for STATUS STAND-BY
    % * 1 for STATUS ACTIVE
    
    v_min = 10; % 36 km/h
    v_max = 36.1; % 130km/h

    % STAND-BY -> OFF; ACTIVE -> OFF;
    if is_fcw_on == 0
        fcw_status = -1;
    else
        % STAND-BY -> ACTIVE
        if velocity >= v_min && velocity <= v_max && gear >= 1 && gear <= 9
            fcw_status = 1;
        % STAND-BY
        else
            fcw_status = 0;
        end
    end

end