classdef Berkeley_FCW_Constants < handle
    %% Constants required by Berkeley FCW Algorithm
    %  When defining maximal deceleration we were deducing from 
    % https://www.skoda-storyboard.com/en/skoda-world/innovation-and-technology/how-do-brakes-learn-how-to-brake/
    % where is stated that mentioned vehicle decelerates 100-0km/h in 33-34m 
    % We used formula from https://www.toppr.com/guides/physics-formulas/deceleration-formula/
    %  where a = (v^2 - u^2)/2s and declared 
    % max_deceleration = 9.32835 m/s^2.
    % 
    % Constants k1,k2,k3 were not mentioned in source paper, as they
    % labeled them as "black-box" meaning defined by their test data. We
    % set values using TTC algorithm's braking distances. They are
    % non-dimensional.
    % d_0 is minimal distance in m


    properties
        d_0 = 10, % m
        k1 = 1,
        k2 = 1,
        k3 = 0,
        max_deceleration = 9.32835 % m/s^2
    end
end

