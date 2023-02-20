classdef TTC_FCW_Constants < handle
    %TTC_FCW_CONSTANTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        brake_eff_max = 0.95,
        brake_eff_min = 0.85,
        t_p_max = 0.75,
        t_p_min = 0.3,
        g = 9.80665,
        critical_ttc = 1.5,
        equivalent_mass_coefficient = 1.04,
        air_density = 1.18,
        aerodynamic_resistance_coefficient = 0.45,
        rolling_resistance_coefficient = 0.015
    end
end

