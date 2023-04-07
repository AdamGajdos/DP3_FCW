classdef TTC_FCW_Algo < FCW_Algo
    %% Time-To-Collision FCW Algorithm
    %  doi: 10.1109/ICIEA.2013.6566508.
    %
    % Calculates minimal (ds_min) and maximal(ds_max) safe braking distance
    %   Then the distance between our and leading vehicle / frontal
    %   obstacle is compare to ds_min and ds_max. In calculation the road
    %   condition is also considered.
    % After distance is compared to ds_min and ds_max the specific rule is
    %   applied. These rules define the road situation criticality as
    %   Danger levels. To unify FCW results with results of other 
    %   implemented FCW algorithms the result mapping is as follows:
    %    - Safe - situation status == 1
    %    - Danger level 1,2 - driver should pay higher attention
    %                       - situation status == 0
    %    - Danger level 3 - the actual road situation is dangerous
    %                     - situation status == -1
    %
    %% Return values
    % Return values of this algorithms are as follows:
    %
    % * (-1) algorithm evaluates actual situation as dangerous
    % 
    % * (0)  algorithm evaluates actual situation as driver should pay bigger attention to road situation
    % 
    % * (1)  algorithm evaluates actual situation as safe

    properties (Constant, Access = private)
        Algorithm_Constants = TTC_FCW_Constants;
    end
    
    methods
        function obj = TTC_FCW_Algo()
        end
    end

    methods(Static, Access = private)
        function d_br = define_braking_distance(brake_efficiency, road_adhesion, area, steep, weight, angle, velocity)
            c_ae = (TTC_FCW_Algo.Algorithm_Constants.air_density * area * TTC_FCW_Algo.Algorithm_Constants.aerodynamic_resistance_coefficient) / 2;

            sign = 1;
            if steep == Road_Steep.DOWNHILL
                sign = -1;
            end
            
            d_br = (TTC_FCW_Algo.Algorithm_Constants.equivalent_mass_coefficient * weight) / (2*TTC_FCW_Algo.Algorithm_Constants.g*c_ae);

            d_br = d_br * log2(abs(1 + (c_ae * velocity ^ 2) / (brake_efficiency * (road_adhesion + TTC_FCW_Algo.Algorithm_Constants.rolling_resistance_coefficient) * weight * cos(angle) + sign * weight * sin(angle))));
        end

        function [d_s_min, d_s_max] = define_safety_braking_distance(velocity, age, area, steep, weight, angle, is_abs_on, road_type, road_condition)
            
            [road_adhesion_min, road_adhesion_max] = TTC_FCW_Algo.set_adhesion_vars(is_abs_on, road_type, road_condition);

            d_b_min = TTC_FCW_Algo.define_braking_distance(TTC_FCW_Algo.Algorithm_Constants.brake_eff_min, road_adhesion_min, area, steep, weight, angle, velocity);

            d_b_max = TTC_FCW_Algo.define_braking_distance(TTC_FCW_Algo.Algorithm_Constants.brake_eff_max, road_adhesion_max, area, steep, weight, angle, velocity);

            [t_r_min, t_r_max] = TTC_FCW_Algo.set_driver_vars(age);
            d_r_min = t_r_min * velocity;
            d_r_max = t_r_max * velocity;

            d_p_min = TTC_FCW_Algo.Algorithm_Constants.t_p_min * velocity;
            d_p_max = TTC_FCW_Algo.Algorithm_Constants.t_p_max * velocity;

            d_s_min = d_b_min + d_r_min + d_p_min;
            d_s_max = d_b_max + d_r_max + d_p_max;
        end

        function [t_r_min, t_r_max] = set_driver_vars(age)
            if age >= 18 && age <= 33
                t_r_min = 0.74;
                t_r_max = 1.1;
            elseif age >= 34 && age <= 47
                t_r_min = 0.75;
                t_r_max = 1.12;
            elseif age >= 48 && age <= 57
                t_r_min = 0.77;
                t_r_max = 1.13;
            elseif age >= 58 && age <= 70
                t_r_min = 0.78;
                t_r_max = 1.17;
            else
                t_r_min = 0.8;
                t_r_max = 1.2;
            end
        end

        function [road_adhesion_min, road_adhesion_max] = set_adhesion_vars(is_abs_on, road_type, road_condition)
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
        end

        function rule_to_apply = apply_rule(rule, ttc, velocity)
            rule_to_apply = "Safe";

            if rule == 1
                if ttc < TTC_FCW_Algo.Algorithm_Constants.critical_ttc
                    rule_to_apply = "Danger Level 3";
                else
                    if velocity > 0
                        rule_to_apply = "Danger Level 1";
                    else
                        rule_to_apply = "Danger Level 2";
                    end
                end
            elseif rule == 2
                if ttc < TTC_FCW_Algo.Algorithm_Constants.critical_ttc
                    rule_to_apply = "Danger Level 2";
                else
                    if velocity > 0
                        rule_to_apply = "Safe";
                    else
                        rule_to_apply = "Danger Level 1";
                    end
                end
            end
        end

        function [sit_status, warning_distance, critical_distance] = define_danger(velocity, age, road_type, road_condition, is_abs_on, area, steep, weight, angle, distance)
            [ds_min, ds_max] = TTC_FCW_Algo.define_safety_braking_distance(velocity, age, area, steep, weight, angle, is_abs_on, road_type, road_condition);
            
            warning_distance = ds_max;%ds_min;
            critical_distance = ds_min;%ds_max;
            
            ttc = distance / velocity;

            if distance < ds_min
                sit_status = TTC_FCW_Algo.apply_rule(1, ttc, velocity);
            elseif distance < ds_max
                sit_status = TTC_FCW_Algo.apply_rule(2, ttc, velocity);
            else
                sit_status = "Safe";
            end
        end

    end

    methods (Static, Access = public)
        function [situation_status, warning_distance, critical_distance] = Resolve(velocity, age, road_type, road_condition, is_abs_on, area, steep, weight, angle, distance)
            
            [tmp_situation_status, warning_distance, critical_distance] = TTC_FCW_Algo.define_danger(velocity, age, road_type, road_condition, is_abs_on, area, steep, weight, angle, distance);
            
            if tmp_situation_status == "Safe"
                situation_status = 1;
            elseif tmp_situation_status == "Danger Level 1" || tmp_situation_status == "Danger Level 2"
                situation_status = 0;   % Attention
            else
                situation_status = -1;  % Dangerous
            end
        end
    end
end

