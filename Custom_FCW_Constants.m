classdef Custom_FCW_Constants < handle
    %% Constants required by Berkeley FCW Algorithm    
    properties
        w_1 = 1.7           % weight for preferred warning distance in weighted avg.
        w_2 = 1.3           % weight for the other warning distance in weighted avg.
        tau_driver = 1      % delay of driver [s]
        tau_system = 0.25   % delay of system [s]
        a_max = 1.08        % m/s^2
        d_0 = 10            % minimal distance in m
    end
end

