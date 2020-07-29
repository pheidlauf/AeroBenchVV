classdef Test_sample < matlab.unittest.TestCase
    %TEST_sample Sample Unit Test
    
    methods
        % This is the constructor for this test class
        % By defining it this way, executing the class runs the tests.
        function obj = Test_sample(runAlone)
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
        % Vars here are accessed via tc.test_var
        test_var
    end
    
    % Functions defined in this method run before each test method.
    methods(TestMethodSetup)
        function setupExample(tc)
            tc.test_var = "foo";
        end
    end
    
    % Functions defined in this method run after each test method.
    methods(TestMethodTeardown)
        function teardownExample(tc)
            tc.test_var = "bar";
        end
    end
    
    % Functions defined in this method are executed as tests
    methods (Test)
        function sample_passing_test(tc)
            % Note that tc is the testCase object.
            littleBunny = baz(tc.test_var);
            tc.verifyTrue(strcmp(littleBunny,"foofoo"))
        end
        
        function sample_failing_test(tc)
            tc.verifyTrue(false)
        end
    end
    
end

% Local Functions (available as helpers for this class only)
function [output] = baz(input)
output = input + input;
end
