# Test Instructions:

### Running unit tests:
1. Add `AeroBench/src/tests/` to your MATLAB path
1. Run [RunAllTests.m](https://community.vdl.afrl.af.mil/tcas/AeroBench/blob/master/src/tests/RunAllTests.m)
1. Review test results in the command window
    - Test results are also saved to the workspace as `results`

### Writing unit tests:
- The file [Test_sample.m](https://community.vdl.afrl.af.mil/tcas/AeroBench/blob/master/src/tests/Test_sample.m) is a sample class demonstrating how to write unit tests. I recommend creating one test class per function/class/file to be tested.

### Information about this test suite:
- The `AeroBench/src/tests/resources` directory is a repository of `.mat` files. 
    - Each `*_baseline.mat` is manually created and contains a given baseline's inputs and expected results
    - This is easier than hand-hard-coding inputs and outputs.
    - A sample script generating a `baseline.mat` file is available in [generateBaselines.m](https://community.vdl.afrl.af.mil/tcas/AeroBench/blob/master/src/tests/helpers/generateBaselines.m).
- The "baseline tests" are meant to ensure that refactors of pre-existing functions does not change the behavior of said functions.
    - They do NOT guarantee that those functions are correct, merely that they maintain their original behavior.

### Useful resources
- https://www.mathworks.com/help/matlab/class-based-unit-tests.html
- https://www.mathworks.com/help/matlab/matlab_prog/write-simple-test-case-using-classes.html
- https://www.mathworks.com/help/matlab/matlab_prog/types-of-qualifications.html
