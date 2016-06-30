tic;
%% The files in this folder implement the Standard Vehicle Model.
% Load parameters and bus definitions of full simulation:
%clear uavsim;
%save('6dofsimdata.mat');
%load('svm_environment.mat');

%% Define model parameters:
uavsim.svm.m = uavsim.cularis.mass;

% Input scalings, defines input saturations:
uavsim.svm.scalings.psi = deg2rad(15);
uavsim.svm.scalings.theta = deg2rad(30);
uavsim.svm.scalings.phi = deg2rad(45);
uavsim.svm.scalings.omega_en_max = uavsim.cularis.k_motor;

uavsim.svm.usat.n_c.upper = [4 2 4]';
uavsim.svm.usat.n_c.lower = [-4 -2 -4]';
g = 9.81;
uavsim.svm.constants.loadfactors2accelerations = diag([g; g; g]);

% Define initial states:
uavsim.svm.p0 = [0 0 0]';
uavsim.svm.v0 = [15 0 0]';

%% Get closed-loop attitude dynamics:
cularis_closedinnerloops_trimscript;
cl = linearize('trimModel_Cularis_turbulentAtmosphere_innerloopsclosed', op);
% Correct naming issue:
cl.u{4} = 'phi_c';
cl.y{4} = 'phi';
phi_cl = cl(4,4);

%% Generate first order models:
uavsim.cularis.linearModels.eulers.ss_nx_cl_forder = svm_reduceorder((T_nxc2nx), 1, 1);
uavsim.cularis.linearModels.eulers.ss_ny_cl_forder = svm_reduceorder(phi_cl, 1, 1);
uavsim.cularis.linearModels.eulers.ss_nz_cl_forder = svm_reduceorder((T_nzc2nz), 1, 1);

%% Generate second order models:
uavsim.cularis.linearModels.eulers.ss_nx_cl_sorder = svm_reduceorder((T_nxc2nx), 2, 1);
% TODO: need tf for ny
uavsim.cularis.linearModels.eulers.ss_ny_cl_sorder = svm_reduceorder(phi_cl, 2, 1);
uavsim.cularis.linearModels.eulers.ss_nz_cl_sorder = svm_reduceorder((T_nzc2nz), 2, 1);
% Test with theta TF:
test = fflib_build2ndOrderLowpass(10, 0.8, 1);
hold off;
step(test);
hold on;
step(uavsim.cularis.linearModels.eulers.ss_theta_cl_reduced);
uavsim.cularis.linearModels.eulers.ss_theta_cl_reduced = test;
uavsim.cularis.linearModels.eulers.ss_nz_cl_sorder = uavsim.cularis.linearModels.eulers.ss_theta_cl_reduced;

close('all');

%% Choices of inner loop dynamics:
svm_par_innerloopmodes_TIMESCALESEPARATION = 1;
svm_par_innerloopmodes_FORDER = 2;
svm_par_innerloopmodes_SORDER = 3;
innerloopmodes = containers.Map();
innerloopmodes(num2str(svm_par_innerloopmodes_TIMESCALESEPARATION)) = 'TIMESCALESEPARATION';
innerloopmodes(num2str(svm_par_innerloopmodes_FORDER)) = 'FORDER';
innerloopmodes(num2str(svm_par_innerloopmodes_SORDER)) = 'SORDER';

%% Build linear models, operating point: cruise flight
uavsim.svm.p0 = [0 0 0]';
uavsim.svm.v0 = [15 0 0]';

% First for timescale separation model:
svm_par_innerloopmode = svm_par_innerloopmodes_TIMESCALESEPARATION;
svm_trim_timescaleseparation;
uavsim.svm.n0 = op.Inputs(1).u;
uavsim.svm.n0 = [0 0 -1];
uavsim.svm.linearmodels_tss.airspeed_15 = linearize('svm_oloop', op);
uavsim.svm.linearmodels_tss.airspeed_15.y = {'px', 'py', 'pz', 'vx', 'vy', 'vz'};


%% Inner loop models of order 1 and  2 need to be fixed, trim is broken, comented out for now.
%% Model with first order inner loops:
%svm_par_innerloopmode = svm_par_innerloopmodes_FORDER;
%uavsim.svm.innerloops.forder.x0 = uavsim.svm.n0;
%svm_trim_forderinnerloops;
%uavsim.svm.linearmodels.forder.airspeed_15 = linearize('svm_oloop', op);
%uavsim.svm.linearmodels.forder.airspeed_15.y = {'px', 'py', 'pz', 'vx', 'vy', 'vz'};

%% Model with second order inner loops:
%svm_par_innerloopmode = svm_par_innerloopmodes_SORDER;
%uavsim.svm.innerloops.sorder.x0 = [uavsim.svm.n0; zeros(3,1)];
% !Trim script broken
%svm_trim_sorderinnerloops;
%uavsim.svm.innerloops.sorder.x0 = op.States(1).x;
%uavsim.svm.linearmodels.sorder.airspeed_15 = linearize('svm_oloop', op);
%uavsim.svm.linearmodels.sorder.airspeed_15.y = {'px', 'py', 'pz', 'vx', 'vy', 'vz'};
%uavsim.svm.linearmodels.sorder.airspeed_15.u = {'nx', 'ny', 'nz', 'nwx', 'nwy', 'nwz'};
%uavsim.svm.linearmodels.sorder.airspeed_15.y = {'px', 'py', 'pz', 'vx', 'vy', 'vz'};
%uavsim.svm.linearmodels.sorder_longi.airspeed_15 = getSubsystem(uavsim.svm.linearmodels.sorder.airspeed_15, ...
%                                                    [1 2 5 6 7 9 10 12], ...
%                                                    [1 3 4 6], ...
%                                                    [1 3 4 6]);
%%
svm_par_innerloopmode = svm_par_innerloopmodes_TIMESCALESEPARATION;
%svm_par_innerloopmode = svm_par_innerloopmodes_FORDER;
%svm_par_innerloopmode = svm_par_innerloopmodes_SORDER;
uavsim.svm.tsample_dryden = 1e-1;
postracking.T = 1e-2;
uavsim.svm.tsample_model = postracking.T;
uavsim.svm.tsample_uav3d = 0.03;
%Kdisc = c2d(K, postracking.T);
svm.tsim = 10;
svm.turbulenceOn = 1;
svm.ambientWindOn = 0;
svm.wakeON = 0;
svm.ambientWind = [3 0 0];
svm.p0 = [1 1 -51];
svm.v0 = [15 0 1];
svm.dp0 = [0 0 0*uavsim.cularis.b];
svm.maneuvergain = 0;
svm.delays.input = 0.00;
My_3D = [-eye(6) eye(6)];
% Turbulence seed: deterministic or random?
%svm.turbulenceseed = '[1 2 3 4]';
svm.turbulenceseed = 'randi(2^6, [1 4])';

disp([mfilename '>> Standard Vehicle Model set up.']);
