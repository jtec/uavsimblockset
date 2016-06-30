% Controller output sampling time:
postracking.T = 0.001;

% Set up Super Twisting controller:
postracking.smode.supertwist_r1 = 10;
postracking.smode.supertwist_r2 = 1;
postracking.smode.supertwist_alpha = 40;
postracking.smode.supertwist_lambda1 = 3;
postracking.smode.supertwist_umax = 100;
clear slBus1;
busInfo = Simulink.Bus.createObject(postracking.smode);

% %% Set up 3rd order HOSM controller:
% % Ideal control law:
% A = [0 1 0;
%     0 0 1;
%     0 0 0];
% B = [0; 0; 1];
% tfin = 5;
% Q = eye(3);
% [M, ~, ~] = care(A, B, Q);
% Gin = [1 1 1];
% zeta0 = 0;
% 
% % Intial plant state:
% postracking.x0 = 1;
% postracking.z0 = 1;
% postracking.Va0 = 0.1;
% postracking.vz0 = 0.1;
% 
% % Switch off HOSM SMC part:
% alpha = 0;
% % Simulate to obtain initial z:
% delta0 = zeros(3,1);
% sim('ISMC');
% z0 = log_z.Data(1, :)';
% % Compute delta_0:
% delta0 = getDelta0(A, B, M, z0, tfin);

% %% Add switched control:
% alpha = 10;
% zeta0 = - log_z.Data(1, 3);
% %sim('ISMC');
