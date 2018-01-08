% F16_GCAS_V2
%
% Files
%   BuildLateralLqrCtrl      - Determines the gains for a decoupled F-16 Linear Quadratic Regulator 
%   BuildLongitudinalLqrCtrl - Determines the gains for a decoupled F-16 Linear Quadratic Regulator 
%   controlledF16            - Returns the LQR-controlled F-16 state derivatives and more
%   EulerAngleExplanation    - Using the flypath3d package, this generates a 3d graphic visualizing 
%   getAutopilotCommands     - Given the time, f16 state, and trim condition, returns desired Nz, ps, 
%   getDefaultSettings       - Returns structs with default settings needed to run the F16 simulations
%   getLinF16                - Given equilibrium trim and controls, returns a linearized state space 
%   Main                     - Configures the settings for, runs, and saves an F-16 simulation.
%   Main_simple              - Configures the settings for, runs, and saves an F-16 simulation.
%   MakeAnimation            - Animates and saves a 3D animation of the aircraft path and attitude
%   MakePicture              - Plots and saves a 3d rendering of the aircraft path and attitude
%   RunF16Sim                - Simulates and analyzes autonomous F-16 maneuvers using ode45
