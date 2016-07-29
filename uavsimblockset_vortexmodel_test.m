% Test and debug script for vortex model proposed in "Modeling of 
% Aerodynamic Coupling Between Aircraft in Close Proximity", Dogan et al.

% Define aircraft parameters as given in appendix:
b = 0.8796;
S = 0.4448;
alpha = deg2rad(8);
theta = alpha;
CL_0 = 0.062369;
CL_alpha = 0.0365;

% For the predecessor:
ID = 1;
acpar.cbar_m = 0.6744;
acpar.ID = 2;
acpar.b_m = b;
acpar.sweep_rad = deg2rad(30);
acpar.dihedral_rad = 0;
acpar.pRear_b_m = [-1/4 0 0]';
acpar.pNose_b_m = [3/4 0 0]';
acpar.posFuselageBottom_b_m = [0 0 0.12/2]';
acpar.posFinTip_b_m = [0 0 -0.12/2]';

acpar.timeOfLastUpdate = 10;
acpar.p_NED_m = [0 0 0]';
acpar.v_NED_mps = [19.8171 0 0]';
% Holy shit, CL_alpha is given w.r.t. alpha in degrees - wtf, author?
acpar.CL = CL_0 + CL_alpha*rad2deg(alpha);
acpar.qAttitude = angle2quat(0, theta, 0)';
acpar.alpha_rad = alpha;
acpar.beta_rad = 0;
VortexSig(ID) = acpar;
% For the follower:
ID = 2;
acpar.ID = ID;
VortexSig(ID) = VortexSig(1);
VortexSig(ID).p_NED_m = [-2*b b 0]';

% Call vortex model for follower aircraft:
%[Vi_bfrozen, omegai_bfrozen, Vi_NED, p0_filament_left, p0_filament_right, dcm_ew, debugport] = uavsimblockset_vortexmodel(...
%   2, 10, VortexSig);

% Run sweep of lateral separations:
dy = -2*b:b/10:2*b;
Vi = [];
omegai = [];
for k=1:length(dy)
    VortexSig(2).p_NED_m = [-2*b dy(k) 0]';
    [Vi_bfrozen, omegai_bfrozen, Vi_NED, p0_filament_left, p0_filament_right, dcm_ew, debugport] = uavsimblockset_vortexmodel(...
    2, 10, VortexSig);
    Vi(:,k) = Vi_bfrozen;
    omegai(:,k) = omegai_bfrozen;    
end

figure(22);
subplot(3,1,1);
plot(dy, Vi(1, :));
ylim([-0.3 0.1]);
grid on;
subplot(3,1,2);
plot(dy, Vi(2, :));
ylim([-0.4 0.3]);
grid on;
subplot(3,1,3);
plot(dy, Vi(3, :));
ylim([-1 2]);
grid on;

figure(23);
subplot(3,1,1);
plot(dy, omegai(1, :));
ylim([-0.3 0.1]);
grid on;
subplot(3,1,2);
plot(dy, omegai(2, :));
ylim([-0.4 0.3]);
grid on;
subplot(3,1,3);
plot(dy, omegai(3, :));
ylim([-1 2]);
grid on;
