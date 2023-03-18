classdef Berkeley_FCW_Constants < handle
    %BERKELEY_FCW_CONSTANTS Summary of this class goes here
    %   Detailed explanation goes here
    
    %  https://www.skoda-storyboard.com/en/skoda-world/innovation-and-technology/how-do-brakes-learn-how-to-brake/
    %  decelerates 100-0km/h in 33-34m https://www.toppr.com/guides/physics-formulas/deceleration-formula/ using
    %  formula a = (v^2 - u^2)/2s -> max_deceleration = 9.32835


    properties
        d_0 = 10,
        k1 = 1, % 0.5,
        k2 = 1,  % 5,
        k3 = 0,
        max_deceleration = 9.32835
    end
end

