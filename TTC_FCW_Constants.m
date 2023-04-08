classdef TTC_FCW_Constants < handle
    %% Constants required by TTC FCW Algorithm
    
    properties
        brake_eff_max = 0.95, % brake efficiency coefficient
        brake_eff_min = 0.85, % brake efficiency coefficient
        t_p_max = 0.75, 
        t_p_min = 0.3,  
        g = 9.80665,    % m/s^2
        critical_ttc = 1.5, % critical time to collision (time to impact 
                            %   when conditions remain the same)
        equivalent_mass_coefficient = 1.04,
        air_density = 1.18,
        aerodynamic_resistance_coefficient = 0.45,
        rolling_resistance_coefficient = 0.015
    end
end

