classdef WaypointFollower < Pilot
    %WaypointFollower This is a Waypoint Follower autopilot.
    
    properties(SetAccess=protected, GetAccess = {?Test_WaypointFollower})
        % Controller constants
        K_alt
        K_h_dot
        K_prop_psi
        K_der_psi
        K_prop_phi
        K_der_phi
        K_vt
        airspeed
        max_Nz_cmd
        u_ol_default
    end
    
    methods
        function self = WaypointFollower(WF_config)
            %WAYPOINT_FOLLOWER Construct an instance of a WaypointFollower
            %Autopilot
            
            % Invoke superclass constructor
            self@Pilot();
            
            % Get default config if none given
            if(nargin < 1)
                WF_config = WaypointFollower.get_default_WF_config();
            end
            
            % Load controller gains into class properties from WF_config.
            self.K_vt = WF_config.K_vt;
            self.airspeed = WF_config.airspeed;
            self.K_alt = WF_config.K_alt;
            self.K_h_dot = WF_config.K_h_dot;
            self.K_prop_psi = WF_config.K_prop_psi;
            self.K_der_psi = WF_config.K_der_psi;
            self.K_prop_phi = WF_config.K_prop_phi;
            self.K_der_phi = WF_config.K_der_phi;
            self.max_Nz_cmd = WF_config.max_Nz_cmd;
            self.u_ol_default = WF_config.u_ol_default;
            
        end
        
        function [u_ol] = getCmds(self, x_f16, waypoints)
            %GETCMDS This function WILL be the code-only implementation of
            %the waypoint following autopilot.
            error('AEROBENCH:WaypointFollower:NotImplememented', ...
                'getCmds is not yet implemented');
        end
        
        function [u_ol] = trackToPoint(self, x_f16, WF_mode, waypoint)
            %GETCMDS returns autopilot's calculated pilot inputs
            % u_ol = [Nz; ps; Ny_r; throttle]
            
            if WF_mode == 0
                [psi_cmd] = WaypointFollower.getWaypointData(...
                    x_f16, waypoint);
                
                % Get desired roll angle given desired heading
                Nz_alt = self.trackAltitudeWingsLevel(x_f16, waypoint(3));
                Nz_roll = self.getNzForLevelTurnOL(x_f16);
                Nz_cmd = Nz_alt + Nz_roll;
                phi_cmd = self.getPhiToTrackHeading(x_f16,psi_cmd);
                ps_cmd = self.trackRollAngle(x_f16,phi_cmd);
                throttle = self.trackAirspeed(x_f16, self.airspeed);
                
                % Create reference vector
                u_ol = [Nz_cmd; ps_cmd; 0; throttle];
            else
                u_ol = self.u_ol_default;
            end
        end
        
        function [u_ol, psi_cmd] = smartTrackToPoint(self, ...
                x_f16, WF_mode, waypoint)
            % u_ol = [Nz; ps; Ny_r; throttle]
            
            if WF_mode == 0
                % Determine desired flight path
                
                [psi_cmd, inclination, horiz_range, vert_range, ...
                    slant_range] = ...
                    WaypointFollower.getWaypointData(x_f16, waypoint);
                
                % Get desired roll angle given desired heading
                phi_cmd = self.getPhiToTrackHeading(x_f16, psi_cmd);
                ps_cmd = self.trackRollAngle(x_f16, phi_cmd);
                
                Nz_cmd = self.trackAltitude(x_f16, waypoint(3));
                throttle = self.trackAirspeed(x_f16, self.airspeed);               
            else
                % Waypoint Following complete: fly level.
                throttle = self.trackAirspeed(x_f16, self.airspeed);
                ps_cmd = self.trackRollAngle(x_f16, 0);
                Nz_cmd = self.trackAltitudeWingsLevel(x_f16, waypoint(3));
                psi_cmd = NaN;
            end
            % Create reference vector
            u_ol = [Nz_cmd; ps_cmd; 0; throttle];
        end
    end
    
    methods(Access = {?Test_Pilots})
        % For testing
        function [u_ol_ref] = spyOnStepImpl(self, x_f16, waypoint)
            [u_ol_ref] = self.stepImpl(x_f16, waypoint, WF_mode);
        end
    end
    
    methods(Access=protected)
        % For matlab.System class
        function [u_ol_ref, psi_cmd] = stepImpl(self, ...
                x_f16, waypoint, WF_mode)
            
            [u_ol_ref, psi_cmd] = self.smartTrackToPoint( ...
                x_f16, WF_mode, waypoint);
            u_ol_ref = u_ol_ref';
        end
        
        function Nz = trackAltitude(self, x_f16, h_cmd)
            % Extract important vars
            h = x_f16(12);              % Altitude      (feet)
            phi = x_f16(4);
            
            % Calculate altitude error (positive => below target alt)
            h_error = h_cmd - h;
            Nz_alt = self.trackAltitudeWingsLevel(x_f16, h_cmd);
            Nz_roll = getNzForLevelTurnOL(self, x_f16);
                
            if h_error > 0
                % Ascend wings level or banked
                Nz = Nz_alt + Nz_roll;
            elseif phi < deg2rad(15)
                % Descend wings (close enough to) level
                Nz = Nz_alt + Nz_roll;
            else
                % Descend in bank (no negative Gs)
                Nz = max(0, Nz_alt + Nz_roll);
            end
        end
        
        function [Nz] = trackAltitudeWingsLevel(self, x_f16, h_cmd)
            % PD control on altitude using Nz [ASSUME WINGS LEVEL]
            
            % Pull out important variables for ease of use
            Vt = x_f16(1);              % Airpeed       (ft/sec)
            h = x_f16(12);              % Altitude      (feet)
            
            % Proportional-Derivative Control
            h_error = h_cmd - h;
            gamma = self.getPathAngle(x_f16);
            h_dot = Vt*sin(gamma); % Calculated, not differentiated
            
            % Calculate Nz command
            Nz = self.K_alt*h_error - self.K_h_dot*h_dot;
        end

        function [Nz] = trackClimbRate(self, x_f16, h_dot_cmd)
            % Proportional control on climb rate using Nz
            
            % Pull out important variables for ease of use
            Vt = x_f16(1);              % Airpeed       (ft/sec)
            
            % Proportional Control
            gamma = self.getPathAngle(x_f16);
            h_dot = Vt*sin(gamma); % Calculated, not differentiated
            
            % Calculate Nz command
            Nz = self.K_h_dot*(h_dot_cmd - h_dot);
        end
        
        function [throttle] = trackAirspeed(self, x_f16, Vt_cmd)
            % Proportional control on airspeed using throttle
            throttle = self.K_vt*(Vt_cmd - x_f16(1));
            throttle = max(min(throttle, 1),0);
        end
        
        function [Nz] = getNzForLevelTurnOL(self, x_f16)
            % Pull g's to maintain altitude during bank based on trig
            
            % Calculate theta
            phi = x_f16(4);
            if(phi <= pi/2 && phi > -pi/2) % If cos(phi) ~= 0, basically
                Nz = 1/cos(phi) - 1; % Keeps plane at altitude
            else
                Nz = 0;
            end
            
            Nz = min(self.max_Nz_cmd, Nz);
            
        end
        
        function [ps] = trackRollAngle(self, x_f16, phi_cmd)
            % PD control on roll angle using stability roll rate
            
            % Pull out important variables for ease of use
            phi = x_f16(4);             % Roll angle    (rad)
            p = x_f16(7);               % Roll rate     (rad/sec)
            
            % Calculate PD control
            ps = (phi_cmd-phi)*self.K_prop_phi - p*self.K_der_phi;
        end
        
        function [phi_cmd] = getPhiToTrackHeading(self, x_f16, psi_cmd)
            % PD Control on heading angle using phi_cmd as control
            
            % Pull out important variables for ease of use
            psi = wrapToPi(x_f16(6));   % Heading angle (rad) zero is north
            r = x_f16(9);               % Heading rate  (rad/s)
            
            % TODO: Handle wrapping
            
            % Calculate PD control
            psi_err = wrapToPi(psi_cmd - psi);
            
            
            phi_cmd = (psi_err)*self.K_prop_psi - r*self.K_der_psi;
            
            % Bound to acceptable bank angles:
            maxBankRad = deg2rad(60);
            phi_cmd = min(max(phi_cmd,-maxBankRad),maxBankRad);
        end
    end
    
    methods(Static)
        function u_ol = getDefaultCmds()
            u_ol = zeros(4,1);
        end
        
        function gamma = getPathAngle(x_f16)
            alpha = x_f16(2);           % AoA           (rad)
            beta = x_f16(3);            % Sideslip      (rad)
            phi = x_f16(4);             % Roll anle     (rad)
            theta = x_f16(5);           % Pitch angle   (rad)
            gamma = asin((cos(alpha)*sin(theta)- ...
                sin(alpha)*cos(theta)*cos(phi))*cos(beta) - ...
                (cos(theta)*sin(phi))*sin(beta));
        end
        
        function [heading, inclination, horiz_range, vert_range, ...
                slant_range] = ...
                getWaypointData(x_f16, waypoint)
            % heading = heading to tgt, equivalent to psi (rad)
            % inclination = polar angle to tgt, equivalent to theta (rad)
            % horiz_range = horizontal range to tgt (ft)
            % vert_range = vertical range to tgt (ft)
            % slant_range = total range to tgt (ft)
            
            [e_pos, n_pos, alt] = WaypointFollower.get_f16_position(x_f16);
            
            
            delta = waypoint - [e_pos, n_pos, alt];
            [~, inclination, slant_range] = cart2sph(...
                delta(1), delta(2), delta(3));
            
            heading = wrapToPi(pi/2 - atan2(delta(2), delta(1)));
            horiz_range = norm(delta(1:2));
            vert_range = norm(delta(3));
        end
        
        function [e_pos, n_pos, alt] = get_f16_position(x_f16)
            % Pull out important variables for ease of use
            n_pos = x_f16(10);          % North position    (ft)
            e_pos = x_f16(11);          % East position     (ft)
            alt = x_f16(12);            % Altitude          (ft)
        end
        
        function WF_config = get_default_WF_config()
            % Returns struct with default WaypointFollower configuration
            
            % Gains for speed control
            WF_config.K_vt = 0.25;
            WF_config.airspeed = 550; % Desired airspeed (KIAS?)
            
            % Gains for altitude tracking
            WF_config.K_alt = 0.005;
            WF_config.K_h_dot = 0.02;
            
            % Gains for heading tracking
            WF_config.K_prop_psi = 5;
            WF_config.K_der_psi = 0.5;
            
            % Gains for roll tracking
            WF_config.K_prop_phi = 0.5;
            WF_config.K_der_phi = 0.9; % old: 0.5
            
            % Gains for Nz
            WF_config.max_Nz_cmd = 4;
            
            % Thresholds for waypoint completion
            WF_config.WP_RANGE_THRESHOLD = 250;
            WF_config.WP_VERT_RANGE_THRESHOLD = 50;
            WF_config.WP_HORIZ_RANGE_THRESHOLD = 250;
            
            % Default control when not waypoint tracking
            WF_config.u_ol_default = [0; 0; 0; 0.3];
        end
    end
end
