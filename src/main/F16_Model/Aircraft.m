classdef Aircraft < matlab.System
    %AIRCRAFT Represents the state, specifics, and limits of an aircraft
    
    properties(Abstract)
        airframe % string
        vMin % ft/s
        vMax % ft/s
        alphaMin % deg
        alphaMax % deg
        betaMax % deg
        altMin % ft AGL
        altMax % ft AGL
        NzMin % G's (0 is steady level)
        NzMax % G's (0 is steady level)
        psMaxAccel % deg/s/s
    end
    
    methods
        function self = Aircraft()          
        end
    end
    
    % Methods not requiring class instantiation
    methods(Abstract, Static)
        inFlightEnvelope = is_in_flight_envelope(AC_config, AC_state)
        flight_limits = get_default_flight_limits()
        ctrl_srfc_limits = get_default_ctrl_srfc_limits()        
    end

end

