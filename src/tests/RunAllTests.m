close all; clear; clc;

% Imports needed for test selection
import matlab.unittest.selectors.HasName;
import matlab.unittest.constraints.ContainsSubstring;
import matlab.unittest.TestRunner;
import matlab.unittest.plugins.TestRunProgressPlugin;

disp("============================================================");
disp("===================  Run All Unit Tests  ===================");
disp("============================================================");
fprintf('\n');

% Define & include aerobench/src directory
aerobench_path = getAeroBenchPath();
addpath(fullfile(aerobench_path,'src','main','utils'));
addAeroBenchPaths(false);

src_root = fullfile(aerobench_path,'src');

% Define test directory
test_root = fullfile(src_root,'tests');

% Select all tests (from test classes)
tests = testsuite(test_root, 'IncludingSubfolders',true);

% Define which tests I don't want to run
excludes = ["sample"; "foooo"];

for i=1:size(excludes,1)
    tests = selectIf(tests,~HasName(ContainsSubstring(excludes(i,:))));
end

% Run tests
results = run(tests);

% Format and print results
disp(table(results));
