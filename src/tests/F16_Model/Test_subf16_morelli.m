classdef Test_subf16_morelli < matlab.unittest.TestCase
    %TEST_SUBF16_MORELLI Unit tests for subf16_morelli
    
    methods
        % This is the constructor for this test class
        % By defining it this way, executing the class runs the tests.
        function obj = Test_subf16_morelli(runAlone)
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
    end
    
    % Functions defined in this method run before each test method.
    methods(TestMethodSetup)
        function setupExample(tc)
            tc.aerobench_path = addAeroBenchPaths(tc.printOn);
        end
    end
    % Functions defined in this method run after each test method.
    methods(TestMethodTeardown)
        function teardownExample(tc)
            return
        end
    end
    
    % Functions defined in this method are executed as tests
    methods (Test)
        function subf16_morelli_returns_correct_baseline_with_u0(tc)
            
            % Define the baseline .mat file
            test_baseline_data = fullfile(tc.aerobench_path, ...
                'src','tests','resources',...
                'subf16_morelli_u0_baseline.mat');
            
            % Define which vars to load
            varsToLoad = {'xd_expected',...
                'Nz_expected',...
                'Ny_expected',...
                'x_f16_input',...
                'u_input'};
            
            % Load baseline vars into test workspace
            load(test_baseline_data,varsToLoad{:});
            
            % Execute function with baseline inputs
            [xd,Nz,Ny] = subf16_morelli(x_f16_input, u_input);
            
            % Verify function outputs match baseline outputs
            tc.verifyEqual(xd, xd_expected)
            tc.verifyEqual(Nz, Nz_expected)
            tc.verifyEqual(Ny, Ny_expected)
        end
        
        function subf16_morelli_returns_correct_baseline_with_u1(tc)
            
            % Define the baseline .mat file
            test_baseline_data = fullfile(tc.aerobench_path, ...
                'src','tests','resources',...
                'subf16_morelli_u1_baseline.mat');
            
            % Define which vars to load
            varsToLoad = {'xd_expected',...
                'Nz_expected',...
                'Ny_expected',...
                'x_f16_input',...
                'u_input'};
            
            % Load baseline vars into test workspace
            load(test_baseline_data,varsToLoad{:});
            
            % Execute function with baseline inputs
            [xd,Nz,Ny] = subf16_morelli(x_f16_input, u_input);
            
            % Verify function outputs match baseline outputs
            tc.verifyEqual(xd, xd_expected)
            tc.verifyEqual(Nz, Nz_expected)
            tc.verifyEqual(Ny, Ny_expected)
        end
        
    end
end
