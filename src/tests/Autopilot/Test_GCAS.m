classdef Test_GCAS < matlab.unittest.TestCase
    %TEST_APPLYCTRLSATLIMITS Unit tests for applyCtrlSatLimits
    
    methods
        % This is the constructor for this test class
        % By defining it this way, executing the class with a single
        % argument of value true, the tests run.
        function self = Test_GCAS(runAlone)
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
        warnState
        GCAS_AP
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
            
            % Build GCAS instance
            tc.GCAS_AP = GCAS();
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
        function GCAS_constructor_sets_defaults(tc)
            % Unique
            tc.verifyClass(tc.GCAS_AP, 'GCAS')
            tc.verifyNotEmpty(tc.GCAS_AP.GCAS_mode)
            tc.verifyEqual(tc.GCAS_AP.Nz_des, 5)
            tc.verifyEqual(tc.GCAS_AP.path_goal, 0)
            tc.verifyEqual(tc.GCAS_AP.GCAS_mode, 0)
        end
        
        function GCAS_constructor_uses_args_if_given(tc)
            path_goal = deg2rad(10);
            Nz_des = 2;
            myGCAS = GCAS(Nz_des, path_goal);
            
            tc.verifyEqual(myGCAS.Nz_des, Nz_des)
            tc.verifyEqual(myGCAS.path_goal, path_goal)
        end
        
        function GCAS_constructor_can_set_mode(tc)
            path_goal = deg2rad(10);
            Nz_des = 2;
            GCAS_mode = 1;
            myGCAS = GCAS(Nz_des, path_goal, GCAS_mode);
            
            tc.verifyEqual(myGCAS.Nz_des, Nz_des)
            tc.verifyEqual(myGCAS.path_goal, path_goal)
            tc.verifyEqual(myGCAS.GCAS_mode, GCAS_mode)
        end
        
        function GCAS_static_getDefaultCmds_returns_4x1_zeros(tc)
            u_ol = tc.GCAS_AP.getDefaultCmds();
            tc.verifySize(u_ol, [4, 1], u_ol);
            tc.verifyEqual(u_ol,zeros(4,1));
        end
        
        function GCAS_getCmds_returns_4x1_array(tc)
            u_ol = tc.GCAS_AP.getCmds(tc.x_f16);
            tc.verifySize(u_ol, [4, 1], u_ol)
        end
        
        function GCAS_stays_mode_1_when_reqs_not_met(tc)
            my_x_f16 = get_x_f16_samples('GCAS_mode_0_to_1');
            path_goal = deg2rad(0);
            Nz_des = 5;
            initialMode = 1;
            myGCAS = GCAS(Nz_des, path_goal, initialMode);
            % Verify setup worked
            tc.verifyEqual(myGCAS.GCAS_mode, initialMode); 
            
            newMode = myGCAS.autosetGCAS_mode(my_x_f16);
            tc.verifyEqual(newMode, myGCAS.GCAS_mode, ...
                'Incorrect function output');
            tc.verifyEqual(myGCAS.GCAS_mode, 1, 'Mode switch failed');         
        end
        
        function GCAS_switches_from_mode_1_to_2_when_wings_level(tc)
            my_x_f16 = get_x_f16_samples('GCAS_mode_1_to_2');
            path_goal = deg2rad(0);
            Nz_des = 5;
            initialMode = 1;
            myGCAS = GCAS(Nz_des, path_goal, initialMode);
            % Verify setup worked
            tc.verifyEqual(myGCAS.GCAS_mode, initialMode); 
            
            newMode = myGCAS.autosetGCAS_mode(my_x_f16);
            tc.verifyEqual(newMode, myGCAS.GCAS_mode, ...
                'Incorrect function output');
            tc.verifyEqual(myGCAS.GCAS_mode, 2, 'Mode switch failed');         
        end
        
        function GCAS_switches_mode_2_to_mode_0_when_nose_up(tc)
            my_x_f16 = get_x_f16_samples('GCAS_mode_2_to_0');
            path_goal = deg2rad(0);
            Nz_des = 5;
            initialMode = 2;
            myGCAS = GCAS(Nz_des, path_goal, initialMode);
            % Verify setup worked
            tc.verifyEqual(myGCAS.GCAS_mode, initialMode); 
            
            newMode = myGCAS.autosetGCAS_mode(my_x_f16);
            tc.verifyEqual(newMode, myGCAS.GCAS_mode, ...
                'Incorrect function output');
            tc.verifyEqual(myGCAS.GCAS_mode, 0, 'Mode switch failed');         
        end
        
        function GCAS_stays_mode_2_when_reqs_not_met(tc)
            my_x_f16 = get_x_f16_samples('GCAS_mode_1_to_2');
            path_goal = deg2rad(0);
            Nz_des = 5;
            initialMode = 2;
            myGCAS = GCAS(Nz_des, path_goal, initialMode);
            % Verify setup worked
            tc.verifyEqual(myGCAS.GCAS_mode, initialMode); 
            
            newMode = myGCAS.autosetGCAS_mode(my_x_f16);
            tc.verifyEqual(newMode, myGCAS.GCAS_mode, ...
                'Incorrect function output');
            tc.verifyEqual(myGCAS.GCAS_mode, 2, 'Mode switch failed');         
        end
        
        function GCAS_switches_mode_0_to_1_when_needed(tc)
            my_x_f16 = get_x_f16_samples('GCAS_mode_0_to_1');
            path_goal = deg2rad(0);
            Nz_des = 5;
            initialMode = 0;
            myGCAS = GCAS(Nz_des, path_goal, initialMode);
            % Verify setup worked
            tc.verifyEqual(myGCAS.GCAS_mode, initialMode); 
            
            newMode = myGCAS.autosetGCAS_mode(my_x_f16);
            tc.verifyEqual(newMode, myGCAS.GCAS_mode, ...
                'Incorrect function output');
            tc.verifyEqual(myGCAS.GCAS_mode, 1, 'Mode switch failed');         
        end
        
        function GCAS_switches_mode_0_to_2_when_needed(tc)
            my_x_f16 = get_x_f16_samples('GCAS_mode_1_to_2');
            path_goal = deg2rad(0);
            Nz_des = 5;
            initialMode = 0;
            myGCAS = GCAS(Nz_des, path_goal, initialMode);
            % Verify setup worked
            tc.verifyEqual(myGCAS.GCAS_mode, initialMode); 
            
            newMode = myGCAS.autosetGCAS_mode(my_x_f16);
            tc.verifyEqual(newMode, myGCAS.GCAS_mode, ...
                'Incorrect function output');
            tc.verifyEqual(myGCAS.GCAS_mode, 2, 'Mode switch failed');         
        end
    end
end
