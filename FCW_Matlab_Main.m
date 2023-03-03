function [situation_status, fcw_warning_distance] = FCW_Matlab_Main(weight, area, age, is_abs_on, driver_delay, system_delay, road_type, road_condition, fcw_algorithm, ...
    longitude, latitude, velocity, relative_velocity, deceleration, distance, steep, angle, run_all_algorithms)
   
    disp("Longitude: " + longitude + " ; Latitude: "  + latitude);

    if run_all_algorithms == 0

        if fcw_algorithm == 0
            [situation_status, fcw_warning_distance] = Berkeley_FCW_Algo.Resolve(velocity, relative_velocity, deceleration, distance, driver_delay);
        elseif fcw_algorithm == 1
            [situation_status, fcw_warning_distance] = Custom_FCW_Algo.Resolve(velocity, relative_velocity,road_type, road_condition, is_abs_on, driver_delay, system_delay, distance);
        elseif fcw_algorithm == 2
            [situation_status, fcw_warning_distance] = TTC_FCW_Algo.Resolve(velocity, age, road_type, road_condition, is_abs_on, area, steep, weight, angle, distance);
        else
            error("Unsupported FCW algorithm");
        end

    else
        
        [berkeley_situation_status, berkeley_fcw_warning_distance] = Berkeley_FCW_Algo.Resolve(velocity, relative_velocity, deceleration, distance, driver_delay);
        [custom_situation_status, custom_fcw_warning_distance] = Custom_FCW_Algo.Resolve(velocity, relative_velocity,road_type, road_condition, is_abs_on, driver_delay, system_delay, distance);
        [ttc_situation_status, ttc_fcw_warning_distance] = TTC_FCW_Algo.Resolve(velocity, age, road_type, road_condition, is_abs_on, area, steep, weight, angle, distance);
        
        fcw_warning_distance = [berkeley_fcw_warning_distance, custom_fcw_warning_distance, ttc_fcw_warning_distance];

        situation_status = [berkeley_situation_status, custom_situation_status, ttc_situation_status];

    end


    
end