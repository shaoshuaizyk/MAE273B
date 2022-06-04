function y = autopilot(uu,P)
%
% Linearized controller


    %% Process inputs
    NN = 0; % states
    x       = uu(1+NN);  % inertial North position
    y       = uu(2+NN);  % inertial East position
    z       = uu(3+NN);  % inertial Down position
    u        = uu(4+NN);  % body x velocity
    v        = uu(5+NN);  % body y velocity
    w        = uu(6+NN);  % body z velocity
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    psi      = uu(9+NN);  % yaw angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
    NN = NN+12; % desired flat outputs
    xx_e     = uu(1+NN:12+NN);  % desired state
    NN = NN+12;
    t        = uu(1+NN);   % time
    
    %% Controller
    
    %
%     u_e = [P.mass*P.gravity/(P.k1*4),P.mass*P.gravity/(P.k1*4),P.mass*P.gravity/(P.k1*4),P.mass*P.gravity/(P.k1*4)]';
 
    % Feedback
    x_c = uu(1:12,1);
    u = (xx_e - x_c);
    if size(P.A, 1) == 4
        disp("Using LQR Gains!");
        u = P.A*u;
    else
        u = u([1,2,3,9], 1);
    end
%     switch size(P.K.D, 1)
%         case 4
%             ;
%         case 2
%             u([2,3], 1) = 0;
%         case 1
%             u = 9;
%         otherwise
%             error("ERROR! Unacceptable number of inputs!");
%     end
    

    %% create outputs
    
    % control outputs
    delta = u;
    % commanded (desired) states
    x_command = xx_e;

    y = [delta; x_command];
 
end
