function [situation_status, fcw_warning_distance, fcw_critical_braking_distance] = FCW_Matlab_Main(weight, area, age, is_abs_on, driver_delay, system_delay, road_type, road_condition, fcw_algorithm, ...
    longitude, latitude, velocity, relative_velocity, deceleration, distance, steep, angle, run_all_algorithms)
    %% Main Script for Simulink Model
    % This represents FCW asistant, which can run multiple FCW algorithms
    %  At the time of creation of this toolbox, the implemented FCW 
    %  algorithms are:
    %  * Berkeley FCW Algorithm
    %  * TTC FCW Algorithm
    %  * Custom (Our) FCW Algorithm
    %
    %% Return values
    % The return values of this script are:
    %
    % * situation_status - represents result of FCW Assistant.
    %
    % * fcw_warning_distance - calculated warning distance in m. Represents the distance after which driver should pay higher attention to road situation.
    %
    % * fcw_critical_braking_distance - calculated critical braking distance in m. Represents the distance after which driver should start immediately.
    %% Mapping for situation status
    % Mapping for statuses is:
    %
    % * -1 for Dangerous situation
    %
    % * 0 for Pay higher attention 
    %
    % * 1 for Safe situation

    if run_all_algorithms == 0

        if fcw_algorithm == 0
            [situation_status, fcw_warning_distance, fcw_critical_braking_distance] = Berkeley_FCW_Algo.Resolve(velocity, relative_velocity, distance, driver_delay);
        elseif fcw_algorithm == 1
            [situation_status, fcw_warning_distance, fcw_critical_braking_distance] = Custom_FCW_Algo.Resolve(velocity, relative_velocity,road_type, road_condition, is_abs_on, driver_delay, system_delay, distance);
        elseif fcw_algorithm == 2
            [situation_status, fcw_warning_distance, fcw_critical_braking_distance] = TTC_FCW_Algo.Resolve(velocity, age, road_type, road_condition, is_abs_on, area, steep, weight, angle, distance);
        else
            error("Unsupported FCW algorithm");
        end

    else
        
        [berkeley_situation_status, berkeley_fcw_warning_distance, berkeley_fcw_critical_braking_distance] = Berkeley_FCW_Algo.Resolve(velocity, relative_velocity, distance, driver_delay);
        [custom_situation_status, custom_fcw_warning_distance, custom_fcw_critical_braking_distance] = Custom_FCW_Algo.Resolve(velocity, relative_velocity,road_type, road_condition, is_abs_on, driver_delay, system_delay, distance);
        [ttc_situation_status, ttc_fcw_warning_distance, ttc_fcw_critical_braking_distance] = TTC_FCW_Algo.Resolve(velocity, age, road_type, road_condition, is_abs_on, area, steep, weight, angle, distance);
        
        fcw_warning_distance = [berkeley_fcw_warning_distance, custom_fcw_warning_distance, ttc_fcw_warning_distance];

        fcw_critical_braking_distance = [berkeley_fcw_critical_braking_distance, custom_fcw_critical_braking_distance, ttc_fcw_critical_braking_distance];

        situation_status = [berkeley_situation_status, custom_situation_status, ttc_situation_status];

    end


    
end