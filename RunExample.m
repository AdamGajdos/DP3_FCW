%% Example of Run
% This is example of running FCW_Matlab_Main

%% Variables initialization
% * *weight* - vehicle weight in kg,
%
% * *area* - frontal area of vehicle in cm^2, 
%
% * *age* - age(years) of driver, 
%
% * *is_abs_on* - indicator wheter ABS is active or not 0(_OFF_), 1(_ON_), 
%
% * *driver_delay* - delay of drivers reaction in s, 
%
% * *system_delay* - delay of systems reaction in s, 
%
% * *road_type* - type of road. Values are: 0(_ASPHALT_),1(_CONCRETE_),2(_SNOW_),3(_ICE_), 
%
% * *road_condition* - condition of road. Values are: 0(_WET), 1(_DRY_), 
%
% * *fcw_algorithm* - selected FCW algorithm. Values are: 0(_Berkeley_),
%  1(_Custom_), 2(_TTC_)
%
% * *longitude* - longitude of current road point (current location), 
%
% * *latitude* - latitude of current road point (current location), 
%
% * *velocity* - velocity of vehicle in m/s, 
%
% * *relative_velocity* - relative velocity (v_lead - v_ours) in m/s, 
%
% * *deceleration* - deceleration of our vehicle in m/s^2, 
%
% * *distance* - distance from frontal obstacle/leading car in m, 
%
% * *steep* - steep of road. Values are: 1(_UPHILL_), -1(_DOWNHILL_), 
%
% * *angle* - angle of steep in °, 
%
% * *run_all_algorithms* - indicator wheter we want to run all algorithms at
% once for comparison matter ( show differences in fcw algorithms's
% behaviour.
%
weight = 1602;
area = 53941; 
age = 18; 
is_abs_on = 1; 
driver_delay = 0.8; 
system_delay = 0.1; 
road_type = 0;
road_condition = 1; 
fcw_algorithm = 0;
longitude = 21.9409; 
latitude = 48.9752; 
velocity = 19.7763; 
relative_velocity = 0.0013; 
deceleration = -0.2698; 
distance = 96.8549; 
steep = 1; 
angle = 0;
run_all_algorithms = 0;

%% Running the code
if run_all_algorithms == 0
    [situation_statuses, warning_distances, critical_distances] = FCW_Matlab_Main(weight, area, age, is_abs_on, driver_delay, system_delay, road_type, road_condition, fcw_algorithm, ...
            longitude, latitude, velocity, relative_velocity, deceleration, distance, steep, angle, run_all_algorithms);
    
    situation_status_berkeley = double(situation_statuses(1));
    berkeley_fcw_warning_distance = double(warning_distances(1));
    berkeley_fcw_critical_braking_distance = double(critical_distances(1));

    fprintf('Situation status: %d\n', situation_status_berkeley);
    fprintf('Warning distance: %d m\n',berkeley_fcw_warning_distance);
    fprintf('Critical braking distance: %d m\n', berkeley_fcw_critical_braking_distance);
end