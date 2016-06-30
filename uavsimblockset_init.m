% Defines variables used throughout the simulation such as model
% parameters, physical constants etc.
%
% Clear old parameter structure:
clear uavsim;
% Load bus signal definitions
load uavsimblockset_busses.mat

% Aerodynamic parameters
uavsim.aero.epsilon =  0.1592;

% Aircraft-specific parameters:
% Lift distribution fidelity:
uavsim.zagi.oswald  =  0.9;
% Wing span
uavsim.zagi.b = 1.4224;
% Wing area
uavsim.zagi.S = 0.2589;
% Wing sweep angle
uavsim.zagi.wingsweep = deg2rad(10);
% Wing dihedral angle
uavsim.zagi.dihedral = 0;
% Propeller circle area:
uavsim.zagi.Sprop = 0.0314;
% Mean aerodynamic chord
uavsim.zagi.c = 0.3302;
% Aspect Ratio
uavsim.zagi.AR = uavsim.zagi.b^2 / uavsim.zagi.S;
% Aerodynamic force application point in the body frame:
uavsim.zagi.aerodynamicCenter_b_m =  [0 0 0]; % m
% Center of gravity in the body frame:
uavsim.zagi.centerOfGravity_b_m = [0 0 0];
uavsim.zagi.inertiaTensor = [0.1147 0      0.0015;
    0      0.0576 0;
    0.0015 0      0.1712];
uavsim.zagi.mass = 1.56;

%%% Lift coefficient %%%
% Zero-alpha lift
uavsim.zagi.CL_0 = 0.09167;
% alpha derivative
uavsim.zagi.CL_alpha = 3.5016;
% Pitch control (elevator) derivative
uavsim.zagi.CL_de = 0.2714;
% Pitch rate derivative
uavsim.zagi.CL_q = 2.8932;

%%% Drag coefficient %%%
% Minimum drag
uavsim.zagi.CD_0 = 0.0254;
% Pitch control (elevator) derivative
uavsim.zagi.CD_de = 0.3045;

%%% Side force coefficient %%%
% Sideslip derivative
uavsim.zagi.CY_beta = -0.07359;

%%% Pitch moment coefficient %%%
% Zero-alpha pitch moment coefficient
uavsim.zagi.Cm_0 = -0.02338;
% alpha derivative
uavsim.zagi.Cm_alpha = -0.5675;
% Pitch control derivative
uavsim.zagi.Cm_de = -0.3254;
% Introduce nonlinearity:
cm_deltae3 = 0.1;

% deltae = deg2rad(-45):deg2rad(1):deg2rad(45);
% for i=1:length(deltae)
%     cmde_linear(i) = uavsim.zagi.Cm_0 + uavsim.zagi.Cm_de * deltae(i);
%     cmde_nonlinear(i) = uavsim.zagi.Cm_0 + uavsim.zagi.Cm_de * deltae(i) + cm_deltae3 * deltae(i)^3;
% end
% hold off;
% plot(rad2deg(deltae), cmde_linear, 'o');
% hold on;
% plot(rad2deg(deltae), cmde_nonlinear, 'o');

uavsim.zagi.Cm_de3 = cm_deltae3;
% Angular rate derivative
uavsim.zagi.Cm_q = -1.3990;

%%% Roll moment coefficient %%%
% Sideslip derivative
uavsim.zagi.Cl_beta = 0.02854;
% Roll control derivative
uavsim.zagi.Cl_da = 0.1682;
% Roll rate derivative
uavsim.zagi.Cl_p = -0.3209;
% Yaw rate derivative
uavsim.zagi.Cl_r = 0.03066;

%%% Yaw moment coefficient %%%
% Sideslip derivative
uavsim.zagi.Cn_beta = - 10 * -0.00040;
% Roll control derivative
uavsim.zagi.Cn_da =  -0.00328;
% Roll rate derivative
uavsim.zagi.Cn_p = 0 * -0.01297;
% Yaw rate derivative
uavsim.zagi.Cn_r = -30 * 0.00434;
% SALify zagi:
% uavsim.zagi.Cn_beta = 0.029;
% uavsim.zagi.Cn_r = -0.026;

% Stall model parameters:
uavsim.zagi.alpha0 = 0.4712;
uavsim.zagi.stall.M = 50;

% Engine parameters:
uavsim.zagi.engine.C_prop = 1;
uavsim.zagi.engine.S_prop = 0.0314;
uavsim.zagi.engine.k_motor = 1.5 * 20;
uavsim.zagi.Tengine = 0.27;

% Default initial position
uavsim.mojaveairport_lla_degdegm = [35.043653 -118.129471 50]';
uavsim.mojaveairport_ecef_m = lla2ecef(uavsim.mojaveairport_lla_degdegm')';
uavsim.originNED_ECEF_radradm  = [deg2rad(uavsim.mojaveairport_lla_degdegm(1:2)); 0];

% Constants:
uavsim.g = 9.81;

% Load Cularis UAS parameters:
CULARIS_coefficients;

% Supergee coefficients:
% Aircraft-specific parameters:
% Lift distribution fidelity:
uavsim.supergee.oswald  =  0.9;
% Wing span
uavsim.supergee.b = 1.499;
% Wing area
uavsim.supergee.S = 0.2167;
% Mean aerodynamic chord
uavsim.supergee.c = 0.1397;
% Aspect Ratio
uavsim.supergee.AR = uavsim.supergee.b^2 / uavsim.supergee.S;
% Aerodynamic force application point in the body frame:
uavsim.supergee.aerodynamicCenter_b_m =  [-0.0906 0 0]; % m
% Center of gravity in the body frame:
uavsim.supergee.centerOfGravity_b_m = [-0.0762 0 0];

 Ixx       =  0.165803e-01;                         
 Iyy       =  0.113692e-01;                         
 Izz       =  0.278108e-01;                          
 Ixy       =  0.304560e-10;                          
 Iyz       = -0.135360e-10;                          
 Izx       = -0.362168e-03;
 Iyx = Ixy;
 Izy = Iyz;
 Ixz = Izx;

uavsim.supergee.inertiaTensor = ...
    [Ixx Ixy Iyz;
     Ixy Iyy Iyz;
     Ixz Iyz Izz];

uavsim.supergee.mass = 0.231;

%%% Lift coefficient %%%
% Zero-alpha lift
uavsim.supergee.CL_0 = 0.40780;
% alpha derivative
uavsim.supergee.CL_alpha = 5.327151;
% Pitch control (elevator) derivative
uavsim.supergee.CL_de = 0.006452;
% Pitch rate derivative
uavsim.supergee.CL_q = 7.911367;

%%% Drag coefficient %%%
% Minimum drag
uavsim.supergee.CD_0 = 0.02 + 0.01;
% Pitch control (elevator) derivative
uavsim.supergee.CD_de = 0.000241;

%%% Side force coefficient %%%
% Sideslip derivative
uavsim.supergee.CY_beta = -0.314680;

%%% Pitch moment coefficient %%%
% Zero-alpha pitch moment coefficient
uavsim.supergee.Cm_0 = 0;
% alpha derivative
uavsim.supergee.Cm_alpha =-0.872655;
% Pitch control derivative
uavsim.supergee.Cm_de = rad2deg( -0.029611);
% Angular rate derivative
uavsim.supergee.Cm_q = -18.872787;

%%% Roll moment coefficient %%%
% Sideslip derivative
uavsim.supergee.Cl_beta = -0.109280;
% Roll control derivative
uavsim.supergee.Cl_da = rad2deg(0.010223);
% Roll rate derivative
uavsim.supergee.Cl_p = -0.555102;
% Yaw rate derivative
uavsim.supergee.Cl_r = 0.146448;

%%% Yaw moment coefficient %%%
% Sideslip derivative
uavsim.supergee.Cn_beta = 0.129164;
% Roll control derivative
uavsim.supergee.Cn_da =  -0.000110;
% Roll rate derivative
uavsim.supergee.Cn_p = -0.051430;
% Yaw rate derivative
uavsim.supergee.Cn_r = -0.148217;

% Stall model parameters:
uavsim.supergee.alpha0 = 0.4712;
uavsim.supergee.stall.M = 50;

% Zagi:
% Define airframe geometry for wake vortex model:
uavsim.zagi.sweepAngle_rad = deg2rad(10);
uavsim.zagi.dihedral_rad = deg2rad(0);
uavsim.zagi.longitudinalReferenceLength = uavsim.zagi.c;
uavsim.zagi.verticalReferenceLength = 0.05;
uavsim.zagi.nose_b_m = [0.15 0 0];
uavsim.zagi.fintip_b_m = [0 0 -0.025];

% Do the same for the cularis
uavsim.cularis.dihedral_rad = 0;
uavsim.cularis.wingsweep_rad = deg2rad(0);
uavsim.cularis.longitudinalReferenceLength = uavsim.cularis.c;
uavsim.cularis.verticalReferenceLength = 0.2;
uavsim.cularis.nose_b_m = [0.40 0 0];
uavsim.cularis.fintip_b_m = [0 0 -0.025];

% Sensor mounting points:
uavsim.avionics.zagi.ahrs.p_b_m = [0 0 0];
uavsim.avionics.zagi.gps.pAntenna_b_m = [-0.3 0 0]';
uavsim.avionics.totaltransportdelay_s = 0.01;

% Actuator limits:
uavsim.avionics.actuators.HS55.omega0 = 10*2*pi;
uavsim.avionics.actuators.HS55.damping = 0.8;
%[uavsim.control.padePWM.num, uavsim.control.padePWM.den] = asymPade(uavsim.avionics.totaltransportdelay_s, 3);
uavsim.avionics.actuators.HS55.ratelimit = deg2rad(60)/0.18;

% These simulink models are used for trimming and generation of linear models:
uavsim.cularis.trimmodel = 'uavsimblockset_trimModel_Cularis_turbulentAtmosphere';

disp([mfilename '>> Intialized uavsim blockset - you''re ready to fly.']);