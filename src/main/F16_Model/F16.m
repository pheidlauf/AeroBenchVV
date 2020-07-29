classdef F16 < Aircraft
    %F16 Represents the state, specifics, and limits of an F16
    %
    %   x_f16 states
    %       x_f16(1)  = air speed, VT                            (ft/s)
    %       x_f16(2)  = angle of attack, alpha                   (rad)
    %       x_f16(3)  = angle of sideslip, beta                  (rad)
    %       x_f16(4)  = roll angle, phi                          (rad)
    %       x_f16(5)  = pitch angle, theta                       (rad)
    %       x_f16(6)  = yaw angle, psi                           (rad)
    %       x_f16(7)  = roll rate, P                             (rad/s)
    %       x_f16(8)  = pitch rate, Q                            (rad/s)
    %       x_f16(9)  = yaw rate, R                              (rad/s)
    %       x_f16(10) = northward horizontal displacement, pn    (ft)
    %       x_f16(11) = eastward horizontal displacement, pe     (ft)
    %       x_f16(12) = altitude, h                              (ft)
    %       x_f16(13) = engine thrust dynamics lag state, pow    (lbs)
    %       ----------------------------------------------------------
    %       x_f16(14) = Integral of Nz error, int_e_Nz           (g's)
    %       x_f16(15) = Integral of Ps_error, int_e_ps           (rad/sec)
    %       x_f16(16) = Integral of Ny_error, int_e_Ny_r         (g's)
    %
    %   Nonlinear f16 controls:
    %       u(1) = throttle command     (0 to 1)
    %       u(2) = elevator command     (deg)
    %       u(3) = aileron command      (deg)
    %       u(4) = rudder command       (deg)
    %
    %   LQR Controls:
    %       u(1) = throttle command     (0 to 1)
    %       u(2) = elevator command     (deg)
    %       u(3) = aileron command      (deg)
    %       u(4) = rudder command       (deg)
    %       ------------------------------------
    %       u(5) = Nz commanded         (g's)
    %       u(6) = roll rate commanded  (deg/s)
    %       u(7) = Ny+r commanded       (g's)
    %
    %   OUTPUT:
    %       xd (16x1)   - units in [rad, rad/s, ft/s, ft, g's, etc...]
    
    properties
        % Flight limits
        airframe
        vMin
        vMax
        alphaMin
        alphaMax
        betaMax
        altMin
        altMax
        NzMin
        NzMax
        psMaxAccel
        % Control sfc limits
        throttleMax
        throttleMin
        elevatorMax
        elevatorMin
        aileronMax
        aileronMin
        rudderMax
        rudderMin
    end
    
    methods
        function self = F16(flightLimits, ctrlLimits)
            %F16 Construct an instance of this class
            self@Aircraft();
            self.airframe = 'F-16';
            
            if(nargin < 2)
                ctrlLimits = F16.get_default_ctrl_srfc_limits();
            end
            
            if(nargin < 1)
                flightLimits = F16.get_default_flight_limits();
            end
            
            % Flight Envelope Limits
            self.vMin = flightLimits.vMin;
            self.vMax = flightLimits.vMax;
            self.alphaMin = flightLimits.alphaMin;
            self.alphaMax = flightLimits.alphaMax;
            self.betaMax = flightLimits.betaMax;
            self.altMin = flightLimits.altMin;
            self.altMax = flightLimits.altMax;
            self.NzMin = flightLimits.NzMin;
            self.NzMax = flightLimits.NzMax;
            self.psMaxAccel = flightLimits.psMaxAccel;
            
            % Control Limits
            self.throttleMax = ctrlLimits.throttleMax;
            self.throttleMin = ctrlLimits.throttleMin;
            self.elevatorMax = ctrlLimits.elevatorMax;
            self.elevatorMin = ctrlLimits.elevatorMin;
            self.aileronMax = ctrlLimits.aileronMax;
            self.aileronMin = ctrlLimits.aileronMin;
            self.rudderMax = ctrlLimits.rudderMax;
            self.rudderMin = ctrlLimits.rudderMin;
        end
    end
    
    methods(Access = {?Test_F16})
        function x_f16_dot = spy_on_stepImpl(self, x_f16, u_il)
            x_f16_dot = self.stepImpl(x_f16, u_il);
        end
    end
    
    %% for Simulink
    methods(Access = protected)
        function x_f16_dot = stepImpl(~, x_f16, u_il)
            x_f16_dot = subf16_morelli(x_f16(1:13), u_il);
        end
    end
    
    methods(Static)
        %% Flight Envelope Limit Checks
        function inFlightEnvelope = is_in_flight_envelope(fltLims, x_f16)
            inFlightEnvelope = ...
                F16.is_in_airspeed_limits(fltLims, x_f16) & ...
                F16.is_in_alpha_limits(fltLims, x_f16) & ...
                F16.is_in_beta_limits(fltLims, x_f16) & ...
                F16.is_in_altitude_limits(fltLims, x_f16);
        end
        
        function inAltitudeLimits = is_in_altitude_limits(fltLims, x_f16)
           altitude = x_f16(12);
           inAltitudeLimits = ...
               fltLims.altMin <= altitude && altitude <= fltLims.altMax;
        end
        
        function inAirspeedLimits = is_in_airspeed_limits(fltLims, x_f16)
            airspeed = x_f16(1);
            inAirspeedLimits = ...
                fltLims.vMin <= airspeed & airspeed <= fltLims.vMax;
        end
        
        function inAlphaLimits = is_in_alpha_limits(fltLims, x_f16)
            alpha = x_f16(2);
            inAlphaLimits = ...
                fltLims.alphaMin <= alpha & alpha <= fltLims.alphaMax;
        end
        
        function inBetaLimits = is_in_beta_limits(fltLims, x_f16)
            beta = x_f16(3);
            inBetaLimits = ...
                -fltLims.betaMax <= beta & beta <= fltLims.betaMax;
        end
        
        function inNzLimits = is_in_Nz_limits(fltLims, x_f16, u_il)
            if nargin < 3
                u_il = zeros(4,1);
            end
            fprintf(u_il)
            Nz = F16.get_derived_states(x_f16, u_il);     
            inNzLimits = fltLims.NzMin <= Nz && Nz <= fltLims.NzMax;
        end
        
        %% Value calculations
        function [Nz, Ps, Ny_r] = get_derived_states(x_f16, u_il)
            % u_ol_actual = [Nz; ps; Ny_r; throttle];
            [~, u_ol_actual] = subf16_morelli(x_f16, u_il);
            Nz = u_ol_actual(1);
            Ps = u_ol_actual(2);
            Ny_r = u_ol_actual(3);
        end
        
        %% Default value generators
        function flightLimits = get_default_flight_limits()
            flightLimits = struct([]);
            flightLimits(1).altMin = 0;         % ft AGL
            flightLimits.altMax = 10000;        % ft AGL
            flightLimits.maneuverTime = 15;     % seconds
            flightLimits.NzMax = 9;             % G's
            flightLimits.NzMin = -2;            % G's
            flightLimits.psMaxAccel = 500;      % deg/s/s
            
            % Note: Alpha, Beta, Vt are hard-coded limits. DO NOT CHANGE
            flightLimits.vMin = 300;            % ft/s
            flightLimits.vMax = 900;            % ft/s
            flightLimits.alphaMin = -10;        % deg
            flightLimits.alphaMax = 45;         % deg
            flightLimits.betaMax = 30;          % deg
        end
        
        function ctrlLimits = get_default_ctrl_srfc_limits()
            ctrlLimits = struct([]);
            ctrlLimits(1).throttleMax = 1;      % AB on for throttle > 0.7
            ctrlLimits.throttleMin = 0;
            ctrlLimits.elevatorMax = 25;
            ctrlLimits.elevatorMin = -25;
            ctrlLimits.aileronMax = 21.5;
            ctrlLimits.aileronMin = -21.5;
            ctrlLimits.rudderMax = 30;
            ctrlLimits.rudderMin = -30;
        end
        
        function maneuverLimits = get_default_maneuver_limits()
            maneuverLimits = struct([]);
            maneuverLimits(1).MaxBank = 60;     % For turning maneuvers
            maneuverLimits.NzMax = 9;
            maneuverLimits.NzMin = -1;
        end
        
    end
end
