function verification_status = FCW_ISO_Verification(fcw_result, distance, velocity, relative_velocity, fcw_state, deceleration, min_distance, system_delay, deceleration_lead)
     % Verify result of FCW assistant to selected ISO 15623:2013 requirements (we are considering only requirements
     % supporting situation evaluation by FCW assistant, thus we are not considering image processing from camera, etc.)

     % verification_status values: 
     %  0 - FCW result is in compliant with selected ISO requirements
     %  1 - FCW result do not comply with selected ISO requirements
     %  2 - FCW assistant has FAILED - wrong behavior of assistant

    verification_status = 0;
    
    if fcw_result < 2
        if fcw_state ~= 1 % only FCW ACTIVE should evaluate situation
            verification_status = 2;
        
        else
            distance_clearence_reduction = system_delay * velocity;  % system_delay * velocity -> distance traveled from brake pedal push to braking start
   
            dec_req = deceleration_lead + (relative_velocity^2 / ( 2 * (distance - distance_clearence_reduction)));
        
            dec_req_tresh_hold = 0.68 * 9.81;
        
            min_distance_delta = 0.5; % in m
    
            ttc_tresh_hold = 4; % in s
        
            % Situation when deceleration is sufficient. FCW should not trigger
            % ATTENTION or WARNING/DANGER
            if (deceleration >= dec_req) && fcw_result < 1 % when FCW result -  ATTENTION or WARNING/DANGER
               verification_status = 1;
               return
            end
        
            % Situation when deceleration is not sufficient. FCW should trigger
            % ATTENTION or WARNING.
            if (dec_req > dec_req_tresh_hold) && fcw_result > 0 % when FCW result -  SAFE
               verification_status = 1;
               return
            end
        
            % Situation when leading vehicle is about to card to our driving lane. 
            % Assuming that direction signal is ON. Vehicle should not trigger
            % warning
            if (relative_velocity - velocity) > velocity && distance >= (min_distance - min_distance_delta) && distance <= (min_distance + min_distance_delta) && fcw_result < 1 % when FCW result is ATTENTION or WARNING/DANGER
                verification_status = 1;
                return
            end
        
            % Situation when there should be enough time to react. FCW should not
            % trigger warning
            TTC = velocity / distance;
            if TTC > ttc_tresh_hold && fcw_result < 1 % when FCW result is ATTENTION or WARNING/DANGER
                verification_status = 1;
                return
            end
        end

    else
        if fcw_state == 1 % when FCW ACTIVE then it should evaluate situation
            verification_status = 2;
        end
    end
end