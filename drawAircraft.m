
function drawAircraft(uu,V,F,patchcolors)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    persistent vehicle_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
%     persistent input_pn
%     persistent input_pe
%     persistent input_pd
%     persistent input_phi
%     persistent input_theta
%     persistent input_psi
%     persistent cur_pn
%     persistent cur_pe
%     persistent cur_pd
%     persistent cur_phi
%     persistent cur_theta
%     persistent cur_psi
%     persistent last_t
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
%         input_pn = pn;
%         input_pe = pe;
%         input_pd = pd;
%         input_phi = phi;
%         input_theta = theta;
%         input_psi = psi;
%         cur_pn = pn;
%         cur_pe = pe;
%         cur_pd = pd;
%         cur_phi = phi;
%         cur_theta = theta;
%         cur_psi = psi;
%         last_t = t;
        f = figure(1);
        f.Position = [1028,1430,702,567];
        clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-2,2,-2,2,-2,2]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
%         dt = t - last_t;
%         last_t = t;
%         % Check if the input position/angle has been changed, if not,
%         % update the value according to the velocities; otherwise, truncate
%         % to the input value
%         if input_pn == pn
%             cur_pn = cur_pn + u*dt;
%         else
%             cur_pn = pn;
%             input_pn = pn;
%         end
%         if input_pe == pe
%             cur_pe = cur_pe + v*dt;
%         else
%             cur_pe = pe;
%             input_pe = pe;
%         end
%         if input_pd == pd
%             cur_pd = cur_pd + w*dt;
%         else
%             cur_pd = pd;
%             input_pd = pd;
%         end
% 
%         % Rotation and rotation are not in the same frame
%         angular_vel_matrix = [1, sin(cur_phi)*tan(cur_theta), cos(cur_phi)*tan(cur_theta);...
%                               0, cos(cur_phi), -sin(cur_phi); 
%                               0, sin(cur_phi)*sec(cur_theta), cos(cur_phi)*sec(cur_theta)];
%         angular_vel = angular_vel_matrix * [p; q; r];
%         if input_phi == phi
%             cur_phi = mod(cur_phi + angular_vel(1)*dt, 2*pi);
%         else
%             cur_phi = phi;
%             input_phi = phi;
%         end
%         if input_theta == theta
%             cur_theta = mod(cur_theta + angular_vel(2)*dt, 2*pi);
%         else
%             cur_theta = theta;
%             input_theta = theta;
%         end
%         if input_psi == psi
%             cur_psi = mod(cur_psi + angular_vel(3)*dt, 2*pi);
%         else
%             cur_psi = psi;
%             input_psi = psi;
%         end

        pn = mod(pn+4, 8) - 4;
        pe = mod(pe+4, 8) - 4;
        pd = mod(pd+4, 8) - 4;
        drawVehicleBody(Vertices,Faces,facecolors,...
                        pn, pe, pd, phi, theta, psi,...
                        vehicle_handle);
%                            cur_pn,cur_pe,cur_pd,cur_phi,cur_theta,cur_psi,...
%                            vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  V = translate(V, pn, pe, pd);  % translate vehicle
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle),
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Define the vertices (physical location of vertices
l = 0.175; % Hummingbird
V = [...
    0, 0, 0;...   % pt 1
    l, 0, 0;...   % pt 2
    0, l, 0;...   % pt 3
    -l, 0, 0;...  % pt 4
    0, -l, 0;...  % pt 5
    l+0.5*l*cos(0*pi/4), 0.5*l*sin(0*pi/4), 0;... % pt 6
    l+0.5*l*cos(1*pi/4), 0.5*l*sin(1*pi/4), 0;... % pt 7
    l+0.5*l*cos(2*pi/4), 0.5*l*sin(2*pi/4), 0;... % pt 8
    l+0.5*l*cos(3*pi/4), 0.5*l*sin(3*pi/4), 0;... % pt 9
    l+0.5*l*cos(4*pi/4), 0.5*l*sin(4*pi/4), 0;... % pt 10
    l+0.5*l*cos(5*pi/4), 0.5*l*sin(5*pi/4), 0;... % pt 11
    l+0.5*l*cos(6*pi/4), 0.5*l*sin(6*pi/4), 0;... % pt 12
    l+0.5*l*cos(7*pi/4), 0.5*l*sin(7*pi/4), 0;... % pt 13
    0.5*l*cos(0*pi/4), l+0.5*l*sin(0*pi/4), 0;... % pt 14
    0.5*l*cos(1*pi/4), l+0.5*l*sin(1*pi/4), 0;... % pt 15
    0.5*l*cos(2*pi/4), l+0.5*l*sin(2*pi/4), 0;... % pt 16
    0.5*l*cos(3*pi/4), l+0.5*l*sin(3*pi/4), 0;... % pt 17
    0.5*l*cos(4*pi/4), l+0.5*l*sin(4*pi/4), 0;... % pt 18
    0.5*l*cos(5*pi/4), l+0.5*l*sin(5*pi/4), 0;... % pt 19
    0.5*l*cos(6*pi/4), l+0.5*l*sin(6*pi/4), 0;... % pt 20
    0.5*l*cos(7*pi/4), l+0.5*l*sin(7*pi/4), 0;... % pt 21
    -l+0.5*l*cos(0*pi/4), 0.5*l*sin(0*pi/4), 0;... % pt 22
    -l+0.5*l*cos(1*pi/4), 0.5*l*sin(1*pi/4), 0;... % pt 23
    -l+0.5*l*cos(2*pi/4), 0.5*l*sin(2*pi/4), 0;... % pt 24
    -l+0.5*l*cos(3*pi/4), 0.5*l*sin(3*pi/4), 0;... % pt 25
    -l+0.5*l*cos(4*pi/4), 0.5*l*sin(4*pi/4), 0;... % pt 26
    -l+0.5*l*cos(5*pi/4), 0.5*l*sin(5*pi/4), 0;... % pt 27
    -l+0.5*l*cos(6*pi/4), 0.5*l*sin(6*pi/4), 0;... % pt 28
    -l+0.5*l*cos(7*pi/4), 0.5*l*sin(7*pi/4), 0;... % pt 29
    0.5*l*cos(0*pi/4), -l+0.5*l*sin(0*pi/4), 0;... % pt 30
    0.5*l*cos(1*pi/4), -l+0.5*l*sin(1*pi/4), 0;... % pt 31
    0.5*l*cos(2*pi/4), -l+0.5*l*sin(2*pi/4), 0;... % pt 32
    0.5*l*cos(3*pi/4), -l+0.5*l*sin(3*pi/4), 0;... % pt 33
    0.5*l*cos(4*pi/4), -l+0.5*l*sin(4*pi/4), 0;... % pt 34
    0.5*l*cos(5*pi/4), -l+0.5*l*sin(5*pi/4), 0;... % pt 35
    0.5*l*cos(6*pi/4), -l+0.5*l*sin(6*pi/4), 0;... % pt 36
    0.5*l*cos(7*pi/4), -l+0.5*l*sin(7*pi/4), 0;... % pt 37
    ]';

% define faces as a list of vertices numbered above
  F = [...
        1, 2, 2;...  % right axis
        1, 3, 3;...  % front axis
        1, 4, 4;...  % left axis
        1, 5, 5;...  % back axis
        2, 6, 7;...  % right rotor faces
        2, 7, 8;...
        2, 8, 9;...
        2, 9,10;...
        2,10,11;...
        2,11,12;...
        2,12,13;...
        2,13, 6;...
        3,14,15;...  % front rotor faces
        3,15,16;...
        3,16,17;...
        3,17,18;...
        3,18,19;...
        3,19,20;...
        3,20,21;...
        3,21,14;...
        4,22,23;...  % left rotor faces
        4,23,24;...
        4,24,25;...
        4,25,26;...
        4,26,27;...
        4,27,28;...
        4,28,29;...
        4,29,22;...
        5,30,31;...  % back rotor faces
        5,31,32;...
        5,32,33;...
        5,33,34;...
        5,34,35;...
        5,35,36;...
        5,36,37;...
        5,37,30;...
        ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  facecolors = [...
    myblue;...    % right axis
    myblue;...    % front axis
    myblue;...    % left axis
    myblue;...    % back axis
    myred;...     % right rotor
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...     % front rotor
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...     % left rotor
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...     % back rotor
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    myred;...
    ];
end
  