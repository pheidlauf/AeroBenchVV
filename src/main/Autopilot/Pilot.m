classdef Pilot < handle & matlab.System
    %AUTOPILOT This is the abstract class of a Pilot
    %   Any autopilot, GCAS, or manual controller should be an instance of
    %   a Pilot.
    
%     properties
%         xequil
%         uequil
%         defaultCmds = [0; 0; 0; 0];
%     end
    
%     properties (Dependent)
%         defaultCmds
%     end
    
    methods
        function self = Pilot()
            %AUTOPILOT Construct instance of this class using
            %default values
%             [self.xequil, self.uequil] = getDefaultEquilibrium();
        end
        
%         function cmd = get.defaultCmds(self)
%             cmd = [self.Nz; self.ps; self.Ny_r; self.throttle];
%         end
    end
    
    methods (Abstract)
        u_ol = getCmds(self)
    end

    methods(Abstract, Access = protected)
        y = stepImpl(self,u) % For matlab.System class
    end
end
