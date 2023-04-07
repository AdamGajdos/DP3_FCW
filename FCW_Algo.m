classdef (Abstract) FCW_Algo
    methods (Abstract, Static, Access = public)
        [situation_status, warning_distance, critical_distance] = Resolve()
    end
end

