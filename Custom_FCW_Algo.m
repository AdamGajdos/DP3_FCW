classdef Custom_FCW_Algo < FCW_Algo
    %CUSTOM_FCW_ALGO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant, Access=private)
        Algorithm_Constants = Custom_FCW_Constants;
    end

    methods
        function obj = Custom_FCW_Algo()
        end
    end

    methods(Static, Access = private)

        function friction = define_friction_coef(is_abs_on, road_type, road_condition)
            road_adhesion_min = 1;
            road_adhesion_max = 1;

            if is_abs_on == 1
                if road_type == Road_Type.ASPHALT
                    if road_condition == Road_Condition.DRY
                        road_adhesion_min = 0.8;
                        road_adhesion_max = 0.9;
                    elseif road_condition == Road_Condition.WET
                        road_adhesion_min = 0.5;
                        road_adhesion_max = 0.7;
                    end
                
                elseif road_type == Road_Type.CONCRETE
                    if road_condition == Road_Condition.DRY
                        road_adhesion_min = 0.8;
                        road_adhesion_max = 0.9;
                    elseif road_condition == Road_Condition.WET
                        road_adhesion_min = 0.8;
                        road_adhesion_max = 0.8;
                    end
                
                elseif road_type == Road_Type.SNOW
                    road_adhesion_min = 0.2;
                    road_adhesion_max = 0.2;
                
                elseif road_type == Road_Type.ICE
                    road_adhesion_min = 0.1;
                    road_adhesion_max = 0.1;
                end

            else
                if road_type == Road_Type.ASPHALT
                    if road_condition == Road_Condition.DRY
                        road_adhesion_min = 0.75;
                        road_adhesion_max = 0.75;
                    elseif road_condition == Road_Condition.WET
                        road_adhesion_min = 0.45;
                        road_adhesion_max = 0.6;
                    end
                elseif road_type == Road_Type.CONCRETE
                    if road_condition == Road_Condition.DRY
                        road_adhesion_min = 0.75;
                        road_adhesion_max = 0.75;
                    elseif road_condition == Road_Condition.WET
                        road_adhesion_min = 0.7;
                        road_adhesion_max = 0.7;
                    end
                elseif road_type == Road_Type.SNOW
                    road_adhesion_min = 0.15;
                    road_adhesion_max = 0.15;
               
                elseif road_type == Road_Type.ICE
                    road_adhesion_min = 0.07;
                    road_adhesion_max = 0.07;
                end
            end
            
            friction = (road_adhesion_min + road_adhesion_max)/2;
        end

        function avg = count_weighted_avg(relative_velocity, d_1, d_2)
            % Because different conservativeness of Honda and Mazda we will try to choose the right approach
            % according to actual situation. If relative velocity (v_leading - v_ours) is >= 0 then leading vehicle
            % is speeding/moving faster than we do, so conservative  approach is more suitable, but the other is valid too.
            % Because of that we consider both values, but with different weights.
            % Otherwise, when relative velocity < 0, then we would like to prefer non conservative approach.

            if relative_velocity >= 0
                nominator = d_1 * Custom_FCW_Algo.Algorithm_Constants.w_1 + d_2 * Custom_FCW_Algo.Algorithm_Constants.w_2;
            else
                nominator = d_1 * Custom_FCW_Algo.Algorithm_Constants.w_2 + d_2 * Custom_FCW_Algo.Algorithm_Constants.w_1;
            end
            denominator = Custom_FCW_Algo.Algorithm_Constants.w_1 + Custom_FCW_Algo.Algorithm_Constants.w_2;

            avg = nominator / denominator;
        end

        function d_w = define_warning_distance(velocity, relative_velocity, delay_driver, delay_system)
            % SDA ; a_1 = a_2 = 5.88 m/s^2
            % a_1_sda = 5.88;
            % a_2_sda = 5.88;
            % d_w_sda = velocity * delay_driver + velocity / (2 * a_1_sda) - ((velocity - relative_velocity) ^ 2) / (2 * a_2_sda);
        
            % WANG ; warning distance higher because of neglected traffic efficiency according
            % to work: "Vehicle forward collision warning algorithm based on road friction"
            % a_1 = a_max
            d_w_wang = velocity * delay_driver - relative_velocity * delay_system + ((velocity ^ 2 - (velocity - relative_velocity) ^ 2) / (2 * Custom_FCW_Algo.Algorithm_Constants.a_max)) + Custom_FCW_Algo.Algorithm_Constants.d_0;        
        
            % HONDA
            d_w_h = 2.2 * - relative_velocity + 6.2;

            d_w1 = d_w_h;

            d_w2 = d_w_wang;  % Try also sda, if results will be better

            d_w = Custom_FCW_Algo.count_weighted_avg(relative_velocity, d_w1, d_w2);
        
        end

        function d_br = define_critical_braking_distance(velocity, relative_velocity, road_type, road_condition, is_abs_on)
        
            % HONDA ; a_1 = a_2 = 7.8 m/s^2 ; tau_system = 0.5 s ; tau_braking_time =  1.5s
            a_1_h = 7.8;
            a_2_h = 7.8;
            tau_system_h = 0.5;
            tau_braking_time_h = 1.5;
            v_2 = velocity - relative_velocity;
            ratio = v_2 / a_2_h;
    
            if ratio < tau_braking_time_h
                d_br_h = tau_braking_time_h * relative_velocity + tau_system_h * tau_braking_time_h * a_1_h - (1 / 2) * a_1_h * (tau_system_h ^ 2);
            else
                d_br_h = tau_braking_time_h * velocity - (1 / 2) * a_1_h * ((tau_braking_time_h - tau_system_h) ^ 2) - ((v_2 ^ 2) / (2 * a_2_h));        
            end
            
            % MAZDA ; a_1 = 6 m/s^2 ; a_2 = 8 m/s^2 ; tau_system = 0.1 s ; tau_driver = 0.6 s
            a_1_m = 6;
            a_2_m = 8;
            tau_system_m = 0.1;
            tau_driver_m = 0.6;
            d_br_m = (1 / 2) * ((velocity ^ 2) / a_1_m - ((velocity - relative_velocity) ^ 2) / a_2_m) + velocity * tau_system_m + relative_velocity * tau_driver_m + Custom_FCW_Algo.Algorithm_Constants.d_0;
    
            % According to work "Development of a Collision Avoidance System - Berkeley" Mazda's way is more conservative
            % than Honda. Mazda tries to avoid even extreme cases on the road.
            d_br1 = d_br_m;
            d_br2 = d_br_h;

            d_br = Custom_FCW_Algo.count_weighted_avg(relative_velocity, d_br1, d_br2) * Custom_FCW_Algo.define_friction_coef(is_abs_on, road_type,road_condition);
        end

        function [sit_status, warning_distance] = define_danger(velocity, relative_velocity, road_type, road_condition, is_abs_on, delay_driver, delay_system, distance)
            d_w = Custom_FCW_Algo.define_warning_distance(velocity, relative_velocity, delay_driver, delay_system);
            d_br = Custom_FCW_Algo.define_critical_braking_distance(velocity, relative_velocity, road_type, road_condition, is_abs_on);

            if d_w < d_br
                tmp = d_w;
                d_w = d_br;
                d_br = tmp;
            end

            if(d_w < 0) && (d_br < 0)
                d_w = Custom_FCW_Algo.Algorithm_Constants.d_0 * 2;
                d_br = Custom_FCW_Algo.Algorithm_Constants.d_0;
            
            elseif(d_br < 0)
                d_br = d_w / 2;
            else % d_w < 0
                if d_br > Custom_FCW_Algo.Algorithm_Constants.d_0
                    d_w = 1.5 * d_br; 
                else
                    d_w = 2 * d_br;
                end
            end

            warning_distance = d_w;

            if distance > d_w
                sit_status = 1; %"Safe";
            elseif distance > d_br && distance <= d_w
                sit_status = 0; %"Attention!";
            else
                sit_status = -1; %"Dangerous!";
            end

        end

    end

    methods(Static, Access = public)
        function [situation_status, warning_distance] = Resolve(velocity, relative_velocity, road_type, road_condition, is_abs_on, delay_driver, delay_system, distance)
            [situation_status, warning_distance] = Custom_FCW_Algo.define_danger(velocity, relative_velocity, road_type, road_condition, is_abs_on, delay_driver, delay_system, distance);
        end
    end
end

