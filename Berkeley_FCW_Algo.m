classdef Berkeley_FCW_Algo < FCW_Algo
    %BERKELEY_FCW_ALGO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant, Access=private)
        Algorithm_Constants = Berkeley_FCW_Constants;
    end

    methods
        function obj = Berkeley_FCW_Algo()
        end
    end

    methods(Static, Access = private)
        function dbr = define_d_br(velocity, relative_velocity)
            dbr = Berkeley_FCW_Algo.Algorithm_Constants.k1 * velocity + Berkeley_FCW_Algo.Algorithm_Constants.k2 * relative_velocity + Berkeley_FCW_Algo.Algorithm_Constants.k3;
        end

        function dw = define_d_w(velocity, relative_velocity, deceleration, delay)
            dw = 0.5 * ((velocity^2) / deceleration - ((velocity - relative_velocity) ^ 2) / deceleration) + velocity * delay + Berkeley_FCW_Algo.Algorithm_Constants.d_0;
        end

        function [nonDimValue, warning_distance] = define_danger(velocity, relative_velocity, deceleration, distance, delay)
            d_br = Berkeley_FCW_Algo.define_d_br(velocity, relative_velocity);

            d_w = Berkeley_FCW_Algo.define_d_w(velocity, relative_velocity, deceleration, delay);
            
            warning_distance = d_w;
            nonDimValue = (distance - d_br) / (d_w - d_br);
            
        end
    end
     
     methods(Static, Access = public)
         function [situation_status, warning_distance] = Resolve(velocity, relative_velocity, deceleration, distance, delay)
           
            [non_dimensional_value, warning_distance] = Berkeley_FCW_Algo.define_danger(velocity, relative_velocity, deceleration, distance, delay);

            a = 0.0000001;
            if non_dimensional_value < a
                situation_status = -1; % Dangerous
            elseif non_dimensional_value <= 1
                situation_status = 0;  % Attention
            else
                situation_status = 1;  % Safe
            end
        end
    end
end

