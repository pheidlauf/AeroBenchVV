classdef Test_F16 < matlab.unittest.TestCase
    %TEST_APPLYCTRLSATLIMITS Unit tests for applyCtrlSatLimits
    
    methods
        % This is the constructor for this test class
        % By defining it this way, executing the class with a single
        % argument of value true, the tests run.
        function self = Test_F16(runAlone)
            if(nargin<1)
                runAlone=false;
            end
            
            % Call constructor of superclass
            self@matlab.unittest.TestCase()
            
            % Run this Testcase
            if(runAlone)
                run(self);
            end
        end
    end
    
    % These are variables available to all tests in this class.
    properties(Access=private)
        aerobench_path
        printOn = false
        warnOn = false
        fltLims
        warnState
        myF16
    end
    
    % Functions defined in this method run before each test method.
    methods(TestMethodSetup)
        function setupExample(tc)
            tc.aerobench_path = addAeroBenchPaths(tc.printOn);
            % Save the original warning state
            tc.warnState = warning;
            % Turn off warning displays (access via lastwarn)
            warning('off');
            % Clear the last warning
            lastwarn('','');
            
            % Build F16 instance
            tc.myF16 = F16();
            tc.fltLims = F16.get_default_flight_limits();
        end
    end
    
    % Functions defined in this method run after each test method.
    methods(TestMethodTeardown)
        function teardownExample(tc)
            % Restore the original warning state
            warning(tc.warnState);
        end
    end
    
    % Functions defined in this method are executed as tests
    methods (Test)       
        %% F16 constructors
        function F16_constructor_sets_defaults(tc)
            % Unique
            tc.verifyClass(tc.myF16, 'F16')
            tc.verifyNotEmpty(tc.myF16.airframe)
            tc.verifyEqual(tc.myF16.airframe, 'F-16')
            tc.verifyEqual(tc.myF16.vMin, 300)
            tc.verifyEqual(tc.myF16.vMax, 900)
        end
        
        function F16_constructor_uses_flight_limits_if_given(tc)
            flightLimits = F16.get_default_flight_limits();
            flightLimits.altMin = -1;
            a_new_F16 = F16(flightLimits);
            
            tc.verifyEqual(a_new_F16.altMin, -1)
        end
        
        
        function F16_constructor_uses_ctrl_limits_if_given(tc)
            flightLimits = F16.get_default_flight_limits();
            ctrlLimits = F16.get_default_ctrl_srfc_limits();
            ctrlLimits.rudderMax = pi;
            a_new_F16 = F16(flightLimits, ctrlLimits);
            
            tc.verifyEqual(a_new_F16.rudderMax, pi)
        end
        
        %% is_in_airspeed_limits
        function is_in_airspeed_limits_works(tc)
            flightLimits = F16.get_default_flight_limits();
            
            x_f16 = get_x_f16_samples('default'); % airspeed just right
            inFlightEnvelope = F16.is_in_airspeed_limits(...
                flightLimits, x_f16);
            tc.verifyTrue(inFlightEnvelope)
            
            x_f16(1) = flightLimits.vMax + 1; % too fast
            inFlightEnvelope = F16.is_in_airspeed_limits(...
                flightLimits, x_f16);
            tc.verifyFalse(inFlightEnvelope)
            
            x_f16(1) = flightLimits.vMin - 1; % too slow
            inFlightEnvelope = F16.is_in_airspeed_limits(...
                flightLimits, x_f16);
            tc.verifyFalse(inFlightEnvelope)
        end
        
        %% is_in_alpha_limits
        function is_in_alpha_limits_works(tc)
            flightLimits = F16.get_default_flight_limits();
            
            x_f16 = get_x_f16_samples('default'); % Alpha just right
            inFlightEnvelope = F16.is_in_alpha_limits(...
                flightLimits, x_f16);
            tc.verifyTrue(inFlightEnvelope)
            
            x_f16(2) = flightLimits.alphaMax + 1; % too high
            inFlightEnvelope = F16.is_in_alpha_limits(...
                flightLimits, x_f16);
            tc.verifyFalse(inFlightEnvelope)
            
            x_f16(2) = flightLimits.alphaMin - 1; % too low
            inFlightEnvelope = F16.is_in_alpha_limits(...
                flightLimits, x_f16);
            tc.verifyFalse(inFlightEnvelope)
        end
        
        %% is_in_beta_limits
        function is_in_beta_limits_works(tc)
            flightLimits = F16.get_default_flight_limits();
            
            x_f16 = get_x_f16_samples('default'); % Beta just right
            inFlightEnvelope = F16.is_in_beta_limits(...
                flightLimits, x_f16);
            tc.verifyTrue(inFlightEnvelope)
            
            x_f16(3) = flightLimits.betaMax + 1; % too high
            inFlightEnvelope = F16.is_in_beta_limits(...
                flightLimits, x_f16);
            tc.verifyFalse(inFlightEnvelope)
            
            x_f16(3) = -flightLimits.betaMax - 1; % too low
            inFlightEnvelope = F16.is_in_beta_limits(...
                flightLimits, x_f16);
            tc.verifyFalse(inFlightEnvelope)
        end
        
        %% is_in_Nz_limits
        function is_in_Nz_limits_works(tc)
            x_f16 = get_x_f16_samples('default'); % Nz just right
           
            inLimits = F16.is_in_Nz_limits(tc.fltLims, x_f16);
            tc.verifyTrue(inLimits);
            
            x_f16(2) = -20; % Negative alpha
            x_f16(8) = -15; % Negative pitch rate
            u_il = [0.5; -20; 0; 0];
           
            inLimits = F16.is_in_Nz_limits(tc.fltLims, x_f16, u_il);
            tc.verifyFalse(inLimits);
        end

        %% is_in_flight_envelope
        function is_in_flight_envelope_checks_airspeed(tc)
            flightLimits = F16.get_default_flight_limits();
            
            x_f16 = get_x_f16_samples('default');
            x_f16(1) = flightLimits.vMax + 1;   % too high
            
            inFlightEnvelope = F16.is_in_flight_envelope(...
                flightLimits, x_f16);
            tc.verifyFalse(inFlightEnvelope)
        end
        
        function is_in_flight_envelope_checks_alpha(tc)
            flightLimits = F16.get_default_flight_limits();
            
            x_f16 = get_x_f16_samples('default');
            x_f16(2) = flightLimits.alphaMax + 1; % too high
            
            inFlightEnvelope = F16.is_in_flight_envelope(...
                flightLimits, x_f16);
            tc.verifyFalse(inFlightEnvelope)
        end
        
        function is_in_flight_envelope_checks_beta(tc)
            flightLimits = F16.get_default_flight_limits();
            
            x_f16 = get_x_f16_samples('default');
            x_f16(3) = flightLimits.betaMax + 1; % too high
            
            inFlightEnvelope = F16.is_in_flight_envelope(...
                flightLimits, x_f16);
            tc.verifyFalse(inFlightEnvelope)
        end
        
        %% Matlab System
        function stepImpl_returns_same_as_subf16_morelli(tc)
            x_f16 = get_x_f16_samples('default');
            u_il = [0.5; 10; -5; 5];
            expected = subf16_morelli(x_f16, u_il);
            f16 = F16();
            x_f16_dot = f16.spy_on_stepImpl(x_f16, u_il);
            
            tc.verifyEqual(x_f16_dot,  expected);
        end
    end
end
