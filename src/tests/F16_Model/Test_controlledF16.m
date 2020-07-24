classdef Test_controlledF16 < matlab.unittest.TestCase
    %TEST_CONTROLLEDF16 Unit tests for controlledF16

    methods
        % This is the constructor for this test class
        % By defining it this way, executing the class runs the tests.
        function obj = Test_controlledF16(runAlone)
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
        printOn=false
        warnState
    end
    
    % Functions defined in this method run before each test method.
    methods(TestMethodSetup)
        function setupExample(tc)
            tc.aerobench_path = addAeroBenchPaths(tc.printOn);
            % Save the original warning state
            tc.warnState = warning;
            warning('off','AEROBENCH:CTRL:saturated');
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
        function controlledF16_returns_correct_baseline(tc)
            % Define the baseline .mat file
            controlledF16_baseline = fullfile(tc.aerobench_path, ...
                'src','tests','resources','controlledF16_baseline.mat');
            % Define which vars to load
            varsToLoad = {'trial_inputs',...
                'xd_expected',...
                'u_expected',...
                'Nz_expected',...
                'ps_expected',...
                'Ny_r_expected'};
            % Load baseline vars into test workspace
            load(controlledF16_baseline,varsToLoad{:});
            
            % Call controlledF16 with the baseline inputs
            [ xd, u, Nz, ps, Ny_r ] = controlledF16( ...
                trial_inputs.t, ...
                trial_inputs.x_f16,...
                trial_inputs.xequil,...
                trial_inputs.uequil,...
                trial_inputs.K_lqr,...
                trial_inputs.F16_model,...
                trial_inputs.lin_f16,...
                trial_inputs.flightLimits,...
                trial_inputs.ctrlLimits,...
                trial_inputs.autopilot);
            
            % verify that the results of controlledF16 match the
            % pre-recorded baseline values.
            verifyEqual(tc, xd, xd_expected)
            verifyEqual(tc, u, u_expected)
            verifyEqual(tc, Nz, Nz_expected)
            verifyEqual(tc, ps, ps_expected)
            verifyEqual(tc, Ny_r, Ny_r_expected)
        end
    end
end
