classdef GCAS < Pilot
    %GCAS_AP This is a GCAS-only autopilot. It is always on.
    
    properties(SetAccess=protected, GetAccess = {?Test_GCAS})
        % Controller constants
        GCAS_mode
        eps_phi
        eps_p
        path_goal
        Nz_des
        K_prop
        K_der
    end
    
    methods
        function self = GCAS(Nz_des, path_goal, GCAS_mode)
            %GCAS Construct an instance of a GCAS autopilot
            
            % Invoke superclass constructor
            self@Pilot();
            
            % Set default vals if none given
            if(nargin < 2)
                self.path_goal = deg2rad(0); % Final desired path angle
                self.Nz_des = 5;             % Desired maneuver g's
            else
                self.path_goal = path_goal;
                self.Nz_des = Nz_des;
            end
            
            % Allows setting of GCAS_mode in constructor
            if(nargin == 3)
                self.GCAS_mode = GCAS_mode;
            else
                self.GCAS_mode = 0;
            end
            
            % Define constants needed for control calculation
            % Choose threshold values:
            self.eps_phi = deg2rad(5);   % Max roll angle magnitude before pulling g's
            self.eps_p = deg2rad(5);     % Max roll rate magnitude before pulling g's
            
            % Set Proportional-Derivative Control Gains
            self.K_prop = 4;
            self.K_der = self.K_prop*0.5;
        end
        
        function [u_ol] = getCmds(self, x_f16, GCAS_mode)
            %GETCMDS returns GCAS's calculated pilot inputs
            % u_ol = [Nz; ps; Ny_r; throttle]
            if(nargin < 3)
                GCAS_mode = self.autoselectGCAS_mode(x_f16);
                self.GCAS_mode = GCAS_mode;
            end
            
            switch GCAS_mode
                case 0
                    u_ol = GCAS.getDefaultCmds();
                case 1
                    u_ol = rollWingsLevel(self,x_f16);
                case 2
                    u_ol = pullNoseLevel(self);
                otherwise
                    error('AEROBENCH:GCAS:mode',...
                        'Unknown GCAS_mode: %d', GCAS_mode);
            end
        end
        
        function newMode = autoselectGCAS_mode(self, x_f16)
            %AUTOSETGCAS_MODE sets the GCAS mode based on F-16 state
            % Mode 0: Inactive
            % Mode 1: Rolling until wings level
            % Mode 2: Pulling until nose up
            
            % Evaluate booleans
            wingsAreLevel = GCAS.areWingsLevel(x_f16,self);
            rollRateIsLow = GCAS.isRollRateLow(x_f16,self);
            noseIsHighEnough = GCAS.isNoseHighEnough(x_f16,self);
            
            % Finite state machine implementation
            newMode = self.GCAS_mode; % Initialize with no mode change
            switch self.GCAS_mode
                case 0 % Inactive
                    % ALWAYS ON GCAS:
                    if ~(wingsAreLevel && rollRateIsLow)
                        newMode = 1;
                    elseif ~noseIsHighEnough
                        newMode = 2;
                    end
                case 1 % Rolling until wings level
                    if wingsAreLevel && rollRateIsLow
                        newMode = 2;
                    end
                case 2 % Pulling up until nose at path_goal
                    if noseIsHighEnough
                        newMode = 0;
                    end
                otherwise
                    error('AEROBENCH:GCAS:mode',...
                        'Unknown GCAS_mode: %d', self.GCAS_mode);
            end
        end
        
        function newMode = autosetGCAS_mode(self, x_f16)
            %AUTOSETGCAS_MODE sets the GCAS mode based on F-16 state
            newMode = autoselectGCAS_mode(self, x_f16);
            self.GCAS_mode = newMode;
        end
    end
    
    methods(Access = {?Test_Pilots})
        % For testing
        function [u_ol_ref, GCAS_mode] = spyOnStepImpl(self, x_f16)
            [u_ol_ref, GCAS_mode] = self.stepImpl(x_f16);
        end
    end
    
    methods(Access=protected)
        function u_ol = rollWingsLevel(self, x_f16)
            %ROLLWINGSLEVEL Calculates the pilot inputs needed to roll the
            % wings level
            
            [phi, p, ~, ~] = GCAS.extractDesiredStates(x_f16);
            
            % Determine which angle is "level" (0, 360, 720, etc)
            radsFromWingsLevel = round(phi/(2*pi));
            
            % PD Control until phi == pi*radsFromWingsLevel
            ps = -(phi - (2*pi)*radsFromWingsLevel)*self.K_prop - p*self.K_der;
            
            % Build commands to roll wings level
            u_ol = GCAS.getDefaultCmds();
            u_ol(2) = ps;
        end
        
        function u_ol = pullNoseLevel(self)
            %ROLLWINGSLEVEL Calculates the pilot input to pull nose level
            u_ol = GCAS.getDefaultCmds();
            u_ol(1) = self.Nz_des;
        end
        
        % For matlab.System class
        function [u_ol_ref] = stepImpl(self, x_f16, GCAS_mode)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            [u_ol_ref] = self.getCmds(x_f16, GCAS_mode);
            u_ol_ref = u_ol_ref';
        end
    end
    
    % Methods not requiring class instantiation
    methods(Static)
        function u_ol = getGcasCmds(x_f16, GCAS_mode)
            % Helper method
            [u_ol] = getCmds(x_f16, GCAS_mode);
        end
        
        function u_ol = getDefaultCmds()
            u_ol = zeros(4,1);
        end
        
        function [phi, p, theta, alpha] = extractDesiredStates(x_f16)
            % Pull out important variables for ease of use
            phi = x_f16(4);             % Roll angle    (rad)
            p = x_f16(7);               % Roll rate     (rad/sec)
            theta = x_f16(5);           % Pitch angle   (rad)
            alpha = x_f16(2);           % AoA           (rad)
        end
        
        function noseIsHighEnough  = isNoseHighEnough(x_f16, GCAS_config)
            [~, ~, theta, alpha] = GCAS.extractDesiredStates(x_f16);
            
            % Determine which angle is "level" (0, 360, 720, etc)
            radsFromNoseLevel = round((theta-alpha)/(2*pi));
            
            % Evaluate boolean
            noseIsHighEnough = (theta-alpha) - 2*pi*radsFromNoseLevel > ...
                GCAS_config.path_goal;
        end
        
        function rollRateIsLow  = isRollRateLow(x_f16, GCAS_config)
            [~, p, ~, ~] = GCAS.extractDesiredStates(x_f16);
            
            % Evaluate boolean
            rollRateIsLow = abs(p) < GCAS_config.eps_p;
        end
        
        function wingsAreLevel  = areWingsLevel(x_f16, GCAS_config)
            [phi, ~, ~, ~] = GCAS.extractDesiredStates(x_f16);
            
            % Determine which angle is "level" (0, 360, 720, etc)
            radsFromWingsLevel = round(phi/(2*pi));
            
            % Evaluate boolean
            wingsAreLevel = abs(phi-(2*pi)*radsFromWingsLevel) < GCAS_config.eps_phi;
        end
        
        function aboveFlightDeck = isAboveFlightDeck(x_f16, GCAS_config)
            aboveFlightDeck = x_f16(12) >= GCAS_config.FLIGHT_DECK;
        end
        
        function GCAS_config = get_default_GCAS_config()
            GCAS_config.eps_phi = deg2rad(5);       % Max abs roll angle before pull
            GCAS_config.eps_p = deg2rad(10);        % Max abs roll rate before pull
            GCAS_config.path_goal = deg2rad(0);     % Min path angle before completion
            GCAS_config.K_prop = 4;                 % Proportional control gain
            GCAS_config.K_der = 3;                  % Derivative control gain
            GCAS_config.FLIGHT_DECK = 1000;         % Altitude at which GCAS activates
            GCAS_config.min_pull_time = 2;          % Min duration of pull up
        end
    end
end
