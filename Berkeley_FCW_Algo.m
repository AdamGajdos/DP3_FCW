classdef Berkeley_FCW_Algo < FCW_Algo
 %% Berkeley FCW Algorithm
    % Algorithm developed at Berkeley University 
    %   in 1998 doi: 10.2307/44741070.
    %
    % This algorithm computes warning distance "d_w" and crititical braking
    %   distance "d_br". Then the actual distance from frontal obstacle is
    %   compared to these distances. When distance < d_w then the warning
    %   should be presented to driver. When distance < d_br then driver 
    %   should start braking immidiately or automatic emergency brakes 
    %   should be applied.
    %
    % Return values of this algorithms are as follows:
    %   - (-1) algorithm evaluates actual situation as dangerous
    %   - (0)  algorithm evaluates actual situation as driver should pay
    %           bigger attention to road situation
    %   - (1)  algorithm evaluates actual situation as safe
    
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

        function dw = define_d_w(velocity, relative_velocity, delay)
            % we are asuming that both our and lead car have the same
            % max_deceleration
            dw = 0.5 * ((velocity^2) / Berkeley_FCW_Algo.Algorithm_Constants.max_deceleration - ((velocity - relative_velocity) ^ 2) / Berkeley_FCW_Algo.Algorithm_Constants.max_deceleration) + velocity * delay + Berkeley_FCW_Algo.Algorithm_Constants.d_0;
        end

        function [nonDimValue, warning_distance, critical_distance] = define_danger(velocity, relative_velocity, distance, delay)
            d_br = Berkeley_FCW_Algo.define_d_br(velocity, relative_velocity);

            d_w = Berkeley_FCW_Algo.define_d_w(velocity, relative_velocity, delay);
            
            warning_distance = d_w;
            critical_distance = d_br;
            nonDimValue = (distance - d_br) / (d_w - d_br);
            
        end
    end
     
     methods(Static, Access = public)
         function [situation_status, warning_distance, critical_distance] = Resolve(velocity, relative_velocity, distance, delay)
           
            [non_dimensional_value, warning_distance, critical_distance] = Berkeley_FCW_Algo.define_danger(velocity, relative_velocity, distance, delay);

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

