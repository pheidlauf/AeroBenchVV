classdef Test_applyCtrlSatLimits < matlab.unittest.TestCase
    %TEST_APPLYCTRLSATLIMITS Unit tests for applyCtrlSatLimits
    
    methods
        % This is the constructor for this test class
        % By defining it this way, executing the class runs the tests.
        function obj = Test_applyCtrlSatLimits(runAlone)
            if(nargin<1)
                runAlone=false;
            end
            
            % Call constructor of superclass
            obj@matlab.unittest.TestCase()
            
            % Run this Testcase
            if(runAlone)
                run(obj);
            end
        end
    end
    
    % These are variables available to all tests in this class.
    properties(Access=private)
        aerobench_path
        printOn = false
        warnOn = false
        ctrlLimits
        u_deg_in_limits
        warnState
    end
    
    % Functions defined in this method run before each test method.
    methods(TestMethodSetup)
        function setupExample(tc)
            tc.aerobench_path = addAeroBenchPaths(tc.printOn);
            [ ~, tc.ctrlLimits, ~ ] = getDefaultSettings();
            tc.u_deg_in_limits = [0.5; 5; -5; 10];
            
            % Save the original warning state
            tc.warnState = warning;
            % Turn off warning displays (access via lastwarn)
            warning('off');
            % Clear the last warning
            lastwarn('','');
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
        function applyCtrlSatLimits_does_nothing_if_in_limits(tc)
            u_deg = applyCtrlSatLimits(tc.u_deg_in_limits, ...
                tc.ctrlLimits, tc.warnOn);
            verifyEqual(tc, tc.u_deg_in_limits, u_deg)
        end
        
        function applyCtrlSatLimits_constrains_to_max(tc)
            u_max = [tc.ctrlLimits.ThrottleMax;
                tc.ctrlLimits.ElevatorMaxDeg;
                tc.ctrlLimits.AileronMaxDeg;
                tc.ctrlLimits.RudderMaxDeg];
            u_deg_in = 1.1 * u_max;
            u_deg = applyCtrlSatLimits(u_deg_in, tc.ctrlLimits,...
                tc.warnOn);
            verifyEqual(tc, u_max, u_deg)
        end
        
        function applyCtrlSatLimits_constrains_to_min(tc)
            u_min = [tc.ctrlLimits.ThrottleMin;
                tc.ctrlLimits.ElevatorMinDeg;
                tc.ctrlLimits.AileronMinDeg;
                tc.ctrlLimits.RudderMinDeg];
            u_deg_in = [ -1; 1.1 * u_min(2:end)];
            u_deg = applyCtrlSatLimits(u_deg_in, tc.ctrlLimits, ...
                tc.warnOn);
            verifyEqual(tc, u_min, u_deg)
        end
        
        function applyCtrlSatLimits_flag_if_limiting_occurs(tc)
            u_max = [tc.ctrlLimits.ThrottleMax;
                tc.ctrlLimits.ElevatorMaxDeg;
                tc.ctrlLimits.AileronMaxDeg;
                tc.ctrlLimits.RudderMaxDeg];
            u_deg_in = 1.1 * u_max;
            [~, isSaturated] = applyCtrlSatLimits(u_deg_in, ...
                tc.ctrlLimits, tc.warnOn);
            verifyTrue(tc, isSaturated)
        end
        
        function apsplyCtrlSatLimits_warns_if_warnOn_true(tc)
            u_max = [tc.ctrlLimits.ThrottleMax;
                tc.ctrlLimits.ElevatorMaxDeg;
                tc.ctrlLimits.AileronMaxDeg;
                tc.ctrlLimits.RudderMaxDeg];
            u_deg_in = 1.1 * u_max;
            
            % Calling this should throw a warning
            applyCtrlSatLimits( ...
                u_deg_in, tc.ctrlLimits);
            % Get the id of the (suppressed) warning
            [~, warnid] = lastwarn;
            % verify that the warning id is what we expect
            tc.verifyEqual(warnid,'AEROBENCH:CTRL:saturated');
        end
        
        function apsplyCtrlSatLimits_warn_message_shows_data(tc)
            u_max = [tc.ctrlLimits.ThrottleMax;
                tc.ctrlLimits.ElevatorMaxDeg;
                tc.ctrlLimits.AileronMaxDeg;
                tc.ctrlLimits.RudderMaxDeg];
            u_deg_in = 1.1 * u_max;
            
            % Calling this should throw a warning
            applyCtrlSatLimits( ...
                u_deg_in, tc.ctrlLimits);
            % Get the message of the (suppressed) warning
            [message, ~] = lastwarn;
            % verify that the warning id is what we expect
            tc.verifySubstring(message,'u_deg_in:');
        end
        
        function applyCtrlSatLimits_does_not_warn_if_warnOn_false(tc)
            funCall = @() applyCtrlSatLimits( ...
                tc.u_deg_in_limits, tc.ctrlLimits);
            
            u_deg = tc.verifyWarningFree(funCall);
        end
    end
end
