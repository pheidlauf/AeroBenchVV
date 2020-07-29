classdef Test_WaypointFollower < matlab.unittest.TestCase
    %TEST_APPLYCTRLSATLIMITS Unit tests for applyCtrlSatLimits
    
    methods
        % This is the constructor for this test class
        % By defining it this way, executing the class with a single
        % argument of value true, the tests run.
        function obj = Test_WaypointFollower(runAlone)
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
        warnState
        WF_AP
        x_f16
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
            
            % Build WaypointFollower instance
            tc.WF_AP = WaypointFollower();
            tc.x_f16 = get_x_f16_samples();
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
        
        function WF_constructor_builds_class(tc)
            tc.verifyClass(tc.WF_AP, 'WaypointFollower')      
            tc.verifyEqual(tc.WF_AP.K_alt, 0.005);
        end
        
        function WF_constructor_builds_class_with_args(tc)
            WF_config = WaypointFollower.get_default_WF_config();
            WF_config.K_alt = 0.001;
            temp_WF_AP = WaypointFollower(WF_config);
            tc.verifyClass(temp_WF_AP, 'WaypointFollower')      
            tc.verifyEqual(temp_WF_AP.K_alt, WF_config.K_alt);
        end
        
        function WF_static_getDefaultCmds_returns_4x1_zeros(tc)
            u_ol = tc.WF_AP.getDefaultCmds();
            tc.verifySize(u_ol, [4, 1], u_ol);
            tc.verifyEqual(u_ol,zeros(4,1));
        end
        
%         function WF_static_distanceToWaypoint_returns_correct_value(tc)
%             waypoints = get_waypoint_samples();
%             distance = WaypointFollower.distanceToWaypoint(x_f16,waypoints);
%             tc.verifyEqual(distance,0)
%         end
    end
end
