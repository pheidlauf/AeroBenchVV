function [x_f16] = get_x_f16_samples(scenario)
%GET_X_F16_SAMPLES Summary of this function goes here
%   Detailed explanation goes here

if nargin == 0
    scenario = 'default';
end

switch scenario
    case 'default'
        x_f16 = getDefaultEquilibrium();
    case 'GCAS_mode_0_to_1'
        x_f16 = [ ...
            540;                    % air speed, V
            deg2rad(2);             % angle of attack, alpha
            0;                      % angle of sideslip, beta
            deg2rad(45);            % roll angle, phi
            deg2rad(-72);           % pitch angle, theta
            deg2rad(-45);           % yaw angle, psi
            0;                      % roll rate, P
            0;                      % pitch rate, Q
            0;                      % yaw rate R
            0;                      % North position
            0;                      % East position
            4000;                   % Altitude
            9;                      % Engine power state
            ];
    case 'GCAS_mode_1_to_2'
        x_f16 = [ ...
            540;                    % air speed, V
            deg2rad(2);             % angle of attack, alpha
            0;                      % angle of sideslip, beta
            0;                      % roll angle, phi
            deg2rad(-72);           % pitch angle, theta
            deg2rad(-45);           % yaw angle, psi
            0;                      % roll rate, P
            0;                      % pitch rate, Q
            0;                      % yaw rate R
            0;                      % North position
            0;                      % East position
            4000;                   % Altitude
            9;                      % Engine power state
            ];
    case 'GCAS_mode_2_to_0'
        x_f16 = [ ...
            540;                    % air speed, V
            deg2rad(2);             % angle of attack, alpha
            0;                      % angle of sideslip, beta
            0;                      % roll angle, phi
            deg2rad(10);            % pitch angle, theta
            deg2rad(0);             % yaw angle, psi
            0;                      % roll rate, P
            0;                      % pitch rate, Q
            0;                      % yaw rate R
            0;                      % North position
            0;                      % East position
            4000;                   % Altitude
            9;                      % Engine power state
            ];
    case 'stalled'
        x_f16 = [ ...
            50;                     % air speed, V
            deg2rad(40);            % angle of attack, alpha
            0;                      % angle of sideslip, beta
            0;                      % roll angle, phi
            deg2rad(60);            % pitch angle, theta
            deg2rad(0);             % yaw angle, psi
            0;                      % roll rate, P
            0;                      % pitch rate, Q
            0;                      % yaw rate R
            0;                      % North position
            0;                      % East position
            4000;                   % Altitude
            9;                      % Engine power state
            ];
    otherwise
        error('AEROBENCH:tests:helpers', ...
            'Scenario not defined for: %s', scenario);
end
