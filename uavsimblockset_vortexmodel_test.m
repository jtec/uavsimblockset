% Test and debug script for vortex model proposed in "Modeling of
% Aerodynamic Coupling Between Aircraft in Close Proximity", Dogan et al.
clear VortexSig;
% Define aircraft parameters as given in appendix:
b = 0.8796;
S = 0.4448;
alpha = deg2rad(8);
theta = alpha;
CL_0 = 0.062369;
CL_alpha_perdeg = 0.0365;

% For the predecessor:
ID = 1;
acpar.cbar_m = 0.6744;
acpar.ID = ID;
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
acpar.CL = CL_0 + CL_alpha_perdeg*rad2deg(alpha);
acpar.qAttitude = angle2quat(0, theta, 0)';
acpar.alpha_rad = alpha;
acpar.beta_rad = 0;
acpar.p_NED_m = [0 0 0]';
acpar.v_NED_mps = [15 0 0]';

usecularis = false;
if usecularis
    ac = uavsim.cularis;
    b = ac.b;
    S = ac.S;
    alpha = -0.04533;
    theta = alpha;
    % For the predecessor:
    acpar.cbar_m = 0.6744;
    acpar.b_m = b;
    acpar.sweep_rad = ac.wingsweep_rad;
    ac.dihedral_rad = 0;
    acpar.pRear_b_m = [-3/4 0 0]';
    acpar.pNose_b_m = [1/4 0 0]';
    acpar.posFuselageBottom_b_m = [0 0 0.12/2]';
    acpar.posFinTip_b_m = [0 0 -0.12/2]';
    
    acpar.CL = 0.3395;
    acpar.qAttitude = angle2quat(0, theta, 0)';
    acpar.alpha_rad = alpha;
    acpar.beta_rad = 0;
end
VortexSig(ID) = acpar;

% For the follower:
ID = 2;
acpar.ID = ID;
VortexSig(ID) = VortexSig(1);
VortexSig(ID).ID = ID;
VortexSig(ID).p_NED_m = [-2*b b 0]';

% Run sweep of lateral separations:
dyend = 1.5*(b);
dystart = -1.5*b;
dy = linspace(dystart, dyend, 50);
Vi = [];
omegai = [];
for k=1:length(dy)
    VortexSig(2).p_NED_m = [-2*b dy(k) 0]';
    [Vi_bfrozen, omegai_bfrozen, Vi_NED, p0_filament_left, p0_filament_right, dcm_ew, debugport] = uavsimblockset_vortexmodel(...
        2, 10, VortexSig);
    Vi(:,k) = Vi_bfrozen;
    omegai(:,k) = omegai_bfrozen;
end

%
figure(22);
hold off;
subplot(3,1,1);
plot(dy/b, Vi(1, :));
ylim([-0.3 0.1]);
grid on;
ylabel Wx_eff;
subplot(3,1,2);
plot(dy/b, Vi(2, :));
ylim([-0.4 0.3]);
grid on;
ylabel Wy_eff;
subplot(3,1,3);
plot(dy/b, Vi(3, :));
ylim([-1 2]);
grid on;
ylabel Wz_eff;
xlabel('dy/b')

figure(23);
hold off;
subplot(3,1,1);
plot(dy/b, -omegai(1, :));
ylim([-15 15]);
ylabel p_eff;
grid on;
subplot(3,1,2);
plot(dy/b, -omegai(2, :));
ylim([-2 1]);
grid on;
ylabel q_eff;
subplot(3,1,3);
plot(dy/b, -(omegai(3, :)));
ylim([-3 3]);
grid on;
ylabel r_eff;
xlabel('dy/b')
