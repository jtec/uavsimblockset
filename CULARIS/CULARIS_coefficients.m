
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% cularis:%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lift distribution fidelity:
uavsim.cularis.oswald  =  0.9;
% Wing span
uavsim.cularis.b = 2.61;
% Wing area
uavsim.cularis.S = 0.42;
% Propelle circle area:
uavsim.cularis.Sprop = 0.0314;
% Mean aerodynamic chord
uavsim.cularis.c = 0.1700;
% Aspect Ratio
uavsim.cularis.AR = uavsim.cularis.b^2 / uavsim.cularis.S;
% Aerodynamic force application point in the body frame:
uavsim.cularis.aerodynamicCenter_b_m =  [0 0 0]; % m
% Center of gravity in the body frame:
uavsim.cularis.centerOfGravity_b_m = [0 0 0];
uavsim.cularis.inertiaTensor = [0.168563   0          -0.005536;
                                0          0.129978   0;
                                -0.005536   0          0.296413];
uavsim.cularis.mass = 2.00;

%%% Lift coefficient %%%
% Zero-alpha lift
uavsim.cularis.CL_0 = 0.63;
% alpha derivative
uavsim.cularis.CL_alpha = 6.28;
% Pitch control (elevator) derivative
uavsim.cularis.CL_de = 0.77;
% Pitch rate derivative
uavsim.cularis.CL_q = 10.60;

%%% Drag coefficient %%%
% Parasitic drag FIXME
uavsim.cularis.CD_0 = 0.0254;
% Pitch control (elevator) derivative
uavsim.cularis.CD_de = 0.0;
% Drag due to pithc
uavsim.cularis.CD_q = 0.0;

%%% Side force coefficient %%%
% Sideslip derivative
uavsim.cularis.CY_beta = -0.24;
% Roll derivative
uavsim.cularis.CY_p = 0.01;
% Yaw derivative
uavsim.cularis.CY_r = 0.14;
% Aileron deflefction derivative
uavsim.cularis.CY_da = -0.01;
% zero value:
uavsim.cularis.CY_0 = 0;
% Rudder deflection derivative:
uavsim.cularis.CY_dr = -0.18;

%%% Pitch moment coefficient %%%
% Zero-alpha pitch
uavsim.cularis.Cm_0 = -0.085;
% alpha derivative
uavsim.cularis.Cm_alpha = -1.40;
% Pitch control derivative
uavsim.cularis.Cm_de = -2.48;
% Pitch rate derivative
uavsim.cularis.Cm_q = -19.44;

%%% Roll moment coefficient %%%
% Zero-alpha roll
uavsim.cularis.Cl_0 = 0.0;
% Sideslip derivative
uavsim.cularis.Cl_beta = -0.07;
% Aileron deflection derivative
uavsim.cularis.Cl_da = -0.34;
% Roll rate derivative
uavsim.cularis.Cl_p = -0.62;
% Yaw rate derivative
uavsim.cularis.Cl_r = 0.16;
% Aileron deflection derivative
uavsim.cularis.Cl_dr = -0.0058;


%%% Yaw moment coefficient %%%
% Zero-alpha pitchza
uavsim.cularis.Cn_0 = 0.0;
% Sideslip derivative
uavsim.cularis.Cn_beta = 0.05;
% Aileron deflection derivative
uavsim.cularis.Cn_da =  0.02;
% Roll rate derivative
uavsim.cularis.Cn_p = -0.06;
% Yaw rate derivative
uavsim.cularis.Cn_r = -0.04;
% Rudder deflection derivative
uavsim.cularis.Cn_dr = 0.05;

% Engine parameters:
uavsim.cularis.C_prop = 1;
uavsim.cularis.S_prop = 0.0314;
uavsim.cularis.k_motor = 60;
uavsim.cularis.Tengine = 0.27;

% Camera parameters:
% Focal length in pixels:
uavsim.cularis.GOPROHERO.fov = deg2rad(140);
% Size per pixel [m]:
uavsim.cularis.GOPROHERO.pix2m = 2.2/(1000000);
% Horizontal and vertical pixel chip size [pixel]:
uavsim.cularis.GOPROHERO.chipX = 512;
uavsim.cularis.GOPROHERO.chipY = 512;
% Focal length (f) [m]:
uavsim.cularis.GOPROHERO.f = [(uavsim.cularis.GOPROHERO.chipX/2)/tan(uavsim.cularis.GOPROHERO.fov/2)]*uavsim.cularis.GOPROHERO.pix2m;
% fx and fy [pixels]
uavsim.cularis.GOPROHERO.fx = uavsim.cularis.GOPROHERO.f/uavsim.cularis.GOPROHERO.pix2m;  
uavsim.cularis.GOPROHERO.fy = uavsim.cularis.GOPROHERO.f/uavsim.cularis.GOPROHERO.pix2m;  

uavsim.cularis.GOPROHERO.ox = uavsim.cularis.GOPROHERO.chipX/2;
uavsim.cularis.GOPROHERO.oy = uavsim.cularis.GOPROHERO.chipY/2;

uavsim.cularis.GOPROHERO.C  = [ 0,  uavsim.cularis.GOPROHERO.fx, uavsim.cularis.GOPROHERO.ox, 0;
                               -uavsim.cularis.GOPROHERO.fy, 0,  uavsim.cularis.GOPROHERO.oy, 0;
                                0,  0,  1,  0;
                                0,  0,  0,  1];                             
                            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Compute cularis polar curve:
Va = [4,6,7,8,10,12,14,16,18,20];
vSi = [-0.38,-0.38,-0.38,-0.40,-0.510,-0.72,-1.04,-1.47,-2.03,-2.75];

%plot(Va,vSi,'*')
uavsim.cularis.pC = polyfit(Va,vSi,2);
%plot([5:1:30],polyval(uavsim.cularis.pC,[5:1:30]));