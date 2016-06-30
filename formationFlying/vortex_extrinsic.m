% This function provides functionality to compute approximate
% velocities and wind angular rates induced by the other members of a UAS
% formation. All units are SI (m, kg, s, rad)
%
% Implements wake model presented in "Modeling of Aerodynamic Coupling
%                                     Between Aircraft in Close Proximity"
%                                     Dogan, Venkataramanan, Blake,
%                                     JOURNAL OF AIRCRAFT,
%                                     Vol. 42, No. 4, July–August 2005
%
% TODO Do sanity check of inputs and parameters.
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function [Vi_bfrozen, omegai_bfrozen, Vi_NED, vortexbus_out] = vortex_extrinsic(pNED, vNED, CL, qAttitude, alpha, beta, p, tsim, vortexbus_in)
% TODO This variable should be removed, as state exchange between UAS is done by
% a bus now.
persistent UASs;

% ID of the calling UAS:
ID = p.id;
if isempty(UASs)
    % Allocate memory for aircraft register:
    s0.id = 0;
    s0.cbar = 0;
    s0.b = 0;
    s0.sweep_rad = 0;
    s0.dihedral_rad = 0;
    s0.refLongi = 0;
    s0.nose_b_m = zeros(3,1);
    s0.refVertical = 0;
    s0.fintip_b_m = zeros(3,1);
    s0.p_NED_m = zeros(3,1);
    s0.vGround_NED_m = zeros(3,1);
    s0.CL = 0;
    s0.qAttitude = zeros(4,1);
    s0.alpha = 0;
    s0.beta = 0;
    s0.vi_center_NED = [0 0 0]';
    s0.vi_eff_b = [0 0 0]';
    s0.omegai_eff_b = [0 0 0]';
    s0.checkpoints.center = [0 0 0]';
    s0.checkpoints.nose = zeros(3,1);
    s0.checkpoints.rear = zeros(3,1);
    s0.checkpoints.leftwingtip = zeros(3,1);
    s0.checkpoints.rightwingtip = zeros(3,1);
    s0.checkpoints.fintip = zeros(3,1);
    s0.checkpoints.finbottom = zeros(3,1);
    % Compute unit vectors pointing along the wing:
    s0.leftwingvector = zeros(3,1);
    s0.rightwingvector = zeros(3,1);
    UASs = [s0; s0; s0; s0; s0; s0; s0; s0; s0; s0];
end

% Update register with state of the calling UAS:
UASs(ID).id = p.id;
UASs(ID).cbar = p.cbar_m;
UASs(ID).b = p.b_m;
UASs(ID).sweep_rad = p.sweep_rad;
UASs(ID).dihedral_rad = p.dihedral_rad;
UASs(ID).refLongi = p.longireflength_m;
UASs(ID).nose_b_m = p.pNose_bf_m;
UASs(ID).refVertical = p.verticalRefLength_m;
UASs(ID).fintip_b_m = p.posFinTip_bf_m;

UASs(ID).p_NED_m = pNED;
UASs(ID).vGround_NED_m = vNED;
UASs(ID).CL = CL;
UASs(ID).qAttitude = qAttitude;
UASs(ID).alpha = alpha;
UASs(ID).beta = beta;
UASs(ID).vi_center_NED = [0 0 0]';
UASs(ID).vi_eff_b = [0 0 0]';
UASs(ID).omegai_eff_b = [0 0 0]';

% Select UASs that have a relevant impact on this one:
relevantUASs = selectDominantUASs(ID);
for j=1:length(relevantUASs)
    % UAS inducing flow:
    maximoleader = UASs(relevantUASs(j));
    if ~isempty(maximoleader)
        % Computation fails for two vehicles that are exactly in the same
        % place, as might be case at the very beginning of a simulation:
        % if norm(maximoleader.p_NED_m - UASs(ID).p_NED_m) > 1e-3
        % FIXME: Not reliable, just wait a little as workaround :
        if tsim > 0.1;
            % Compute rotation matrix from the leader's wind frame to the
            % follower's body frame:
            % DCM (wind leader) -> (body leader)
            % Inialize shared values structure:
            stack.DCM_bw_leader = zeros(3);
            stack.DCM_eb_leader = zeros(3);
            stack.DCM_ew_leader = zeros(3);
            stack.DCM_be_follower = zeros(3);
            stack.DCM_windLeader2bodyFollower = zeros(3);
            stack.p0_rightFilament_wleader = zeros(3,1);
            stack.p0_leftFilament_wleader = zeros(3,1);
            stack.p0_rightFilament_bleader = zeros(3,1);
            stack.p0_leftFilament_bleader = zeros(3,1);
            
            stack.DCM_bw_leader = ffb_dcmbody2wind(maximoleader.alpha, maximoleader.beta)';
            % DCM (body leader) -> NED
            stack.DCM_eb_leader = ffb_quat2dcm(maximoleader.qAttitude')';
            % DCM (wind leader) -> NED
            stack.DCM_ew_leader = stack.DCM_eb_leader * stack.DCM_bw_leader;
            % DCM NED -> (body follower)
            stack.DCM_be_follower = ffb_quat2dcm(UASs(ID).qAttitude');
            stack.DCM_windLeader2bodyFollower = stack.DCM_be_follower * stack.DCM_ew_leader;
            % Compute source points of vortex filaments:
            % unit vector that points from the wind frame's origin to the right
            % wing tip:
            % TODO Handling of b not entirely correct.
            stack.p0_rightFilament_bleader = getWingPoint(maximoleader,  0.5 * maximoleader.b * pi/4, 1);
            stack.p0_rightFilament_wleader = stack.DCM_bw_leader' * stack.p0_rightFilament_bleader;
            % The same procedure for the left wing:
            stack.p0_leftFilament_bleader = getWingPoint(maximoleader,  0.5 * maximoleader.b * pi/4, -1);
            stack.p0_leftFilament_wleader = stack.DCM_bw_leader' * stack.p0_leftFilament_bleader;
            
            % Compute bounding points:
            if ~isfield(UASs(ID), 'checkpoints')
                UASs(ID).checkpoints.center = [0 0 0]';
                UASs(ID).checkpoints.nose = UASs(ID).nose_b_m;
                UASs(ID).checkpoints.rear = UASs(ID).nose_b_m + [-UASs(ID).refLongi 0 0]';
                UASs(ID).checkpoints.leftwingtip = getWingPoint(UASs(ID),  0.5 * maximoleader.b, -1);
                UASs(ID).checkpoints.rightwingtip = getWingPoint(UASs(ID),  0.5 * maximoleader.b, 1);
                UASs(ID).checkpoints.fintip = UASs(ID).fintip_b_m;
                UASs(ID).checkpoints.finbottom = UASs(ID).fintip_b_m + [0 0 -UASs(ID).refVertical]';
                % Compute unit vectors pointing along the wing:
                UASs(ID).leftwingvector = fflib_normalize(UASs(ID).checkpoints.leftwingtip);
                UASs(ID).rightwingvector = fflib_normalize(UASs(ID).checkpoints.rightwingtip);
            end
            [test, UASs(ID).vi_center_NED]= getVi(UASs(ID).checkpoints.center, ...
                UASs(ID), maximoleader, ...
                stack);
            
            debugos = false;
            if debugos
                computeInducedV();
            end
            [UASs] = computeEffectiveVAndOmega(UASs, ID, maximoleader, stack);
        end
    end
end

Vi_bfrozen = UASs(ID).vi_eff_b;
omegai_bfrozen = UASs(ID).omegai_eff_b;
Vi_NED = UASs(ID).vi_center_NED;

bp = 0;
end


% Computes the average induced velocity and índuced body rotation rates:
function UASs_out = computeEffectiveVAndOmega(UASs, ID, maximoleader, stack)
vi_center2nose = average(UASs(ID).checkpoints.center, UASs(ID).checkpoints.nose, UASs, ID, maximoleader, stack);
vi_center2rear = average(UASs(ID).checkpoints.center, UASs(ID).checkpoints.rear, UASs, ID, maximoleader, stack);
vi_center2rightwingtip = average(UASs(ID).checkpoints.center, UASs(ID).checkpoints.rightwingtip, UASs, ID, maximoleader, stack);
vi_center2leftwingtip = average(UASs(ID).checkpoints.center, UASs(ID).checkpoints.leftwingtip, UASs, ID, maximoleader, stack);
vi_center2fintip = average(UASs(ID).checkpoints.center, UASs(ID).checkpoints.fintip, UASs, ID, maximoleader, stack);
vi_center2finbottom = average(UASs(ID).checkpoints.center, UASs(ID).checkpoints.finbottom, UASs, ID, maximoleader, stack);

UASs(ID).vi_eff_b(1) = 1/4 * (vi_center2rightwingtip(1) ...
    + vi_center2leftwingtip(1) ...
    + vi_center2fintip(1) ...
    + vi_center2finbottom(1));

UASs(ID).vi_eff_b(2) = 1/4 * (vi_center2nose(2) ...
    + vi_center2rear(2) ...
    + vi_center2fintip(2) ...
    + vi_center2finbottom(2));

UASs(ID).vi_eff_b(3) = 1/4 * (vi_center2rear(3) ...
    + vi_center2nose(3) ...
    + vi_center2rightwingtip(3) ...
    + vi_center2leftwingtip(3));
UASs_out = UASs;
end

% Computes the average induced velocity over one airframe segment
% by integrating between two checkpoints in the body frame.
function vi_av = average(p1, p2, UASs, ID, maximoleader, stack)
% Generate evenly spaced integration points:
ds = 0.1;
l = norm(p1-p2);
steps = 0:ds:l;
unit_p1top2 = fflib_normalize(p2-p1);
vi_av = zeros(3,1);
for i=1:length(steps);
    p = p1 + steps(i) * unit_p1top2;
    [vi_atPoint_b, ~] = getVi(p, UASs(ID), maximoleader, stack);
    vi_av = vi_av + vi_atPoint_b;
end
vi_av = vi_av ./ length(steps);
end


% Computes the induced velocity distribution at discrete points:
function computeInducedV()
b = UASs(ID).b;

vi_alongBodyX = distribution([0 0 0]', [1 0 0]', [-60*b, 20*b]);
vi_alongBodyZ = distribution([0 0 0]', [0 0 1]', [-2*b 2*b]');
vi_alongtheleftwing = distribution([0 0 0]', UASs(ID).leftwingvector, [0 4*b]);
vi_alongtherightwing = distribution([0 0 0]', - UASs(ID).rightwingvector, [0 -4*b]);
vi_alongthewing = [vi_alongtheleftwing vi_alongtherightwing];

% Piotr:
subplot(3,2,1)
plot(vi_alongthewing(1,:), vi_alongthewing(2,:) );
title('Wx along the wing')
axis([-4,4,-0.4,0.3]);
subplot(3,2,2)
plot(vi_alongBodyZ(1,:), vi_alongBodyZ(2,:) );
title('Wx along body z')
axis([-2,2,-0.01,0.06]);

subplot(3,2,3)
plot(vi_alongBodyX(1,:), vi_alongBodyX(3,:) );
title('Wy along body x')
axis([-60,20,-0.2,0.3]);
subplot(3,2,4)
plot(vi_alongBodyZ(1,:), vi_alongBodyZ(3,:) );
title('Wy along body z')
axis([-2,2,-0.4,0.3]);

subplot(3,2,5)
plot(vi_alongBodyX(1,:), vi_alongBodyX(4,:) );
title('Wz along body x')
axis([-60,20,-0.4,0.1]);
subplot(3,2,6)
plot(vi_alongthewing(1,:), vi_alongthewing(4,:) );
title('Wz along the wing')
axis([-4,4,-1.5,2.5]);


end

% Computes the induced velocity distribution over one airframe
% segment defined by a point in the body frame, a unit vector and
% a range.
function vi_dist = distribution(p0, u, r, stack)
% Generate evenly spaced points:
ds = UASs(ID).b/20;
steps = linspace(r(1),r(2), ceil(abs(r(2)-r(1))/ds));
vi_dist = zeros(4,length(steps));
ps = [];
for i=1:length(steps);
    p = p0 + steps(i) * u;
    [vi_dist(2:4, i), ~] = getVi(p, UASs(ID), maximoleader, stack.DCM_ew_leader, stack.DCM_windLeader2bodyFollower, stack.DCM_be_follower, stack.p0_rightFilament_wleader, stack.p0_leftFilament_wleader);
    ps = [ps p];
    vi_dist(1, i) = steps(i)/UASs(ID).b;
end
end

% Computes the body frame coordinates of a point on the wing of the given
% UAS for the given distance from the body frame's origin. The third
% parameter indicates the halfplane the point is supposed to lie in
% (1=right wing, -1=left wing). Helpful e.g. to compute the positions of
% the wing tips for an aicraft featuring dihedral and sweep angle.
function p = getWingPoint(ac, r, hlfpl)
% Compute unit vector pointing from the origin to the wing tip:
wingvector = ffb_angle2dcm((hlfpl)*ac.sweep_rad, (hlfpl)*ac.dihedral_rad, 0, 'ZXY')' * [0 hlfpl 0]';
p = wingvector * r;
end

% Computes the induced velocity vector in the follower's body frame at a
% given point defined in the follower's body frame.
function [vi_b, vi_NED]= getVi(p_bfollower, follower, leader, stack)
% Find point coordinates in the leader's wind frame:
% First compute position of the follower's body frame in the
% leader's wind frame.
% relative position vector in the NED frame:
d_NED = body2NED(p_bfollower, follower.p_NED_m, follower.qAttitude) - leader.p_NED_m;
% rotate to the leader's wind frame:
d_wleader = stack.DCM_ew_leader' * d_NED;
% Transform point:
p_wleader = zeros(3,1);
p_wleader = fflib_frame2frame(p_bfollower, d_wleader, ffb_dcm2quat(stack.DCM_windLeader2bodyFollower)');
% Compute the velocities induced by the right and the left
% filament:
% Get induced velocities:
vi_wleader = computeVi(p_wleader, stack.p0_leftFilament_wleader, [1 0 0]', leader, d_wleader) ...
    + computeVi(p_wleader, stack.p0_rightFilament_wleader, [-1 0 0]', leader, d_wleader);
vi_b = stack.DCM_windLeader2bodyFollower * vi_wleader;
vi_NED = stack.DCM_be_follower' * vi_b;
end


% Computes the induced velocity vector for a given filament that
% starts at the given point in the leader's wind frame.
% The unit vector u_senseofrotation_wleader indicates the sense of
% rotation of the vortex (right hand rule).
function vi_wleader = computeVi(p_wleader, p0_filament_wleader, u_senseofrotation_wleader, leader, d_wleader)
% To compute the norm of the induced velocity, we need the
% longitudinal separation to compute the vortex age tau:
dx = d_wleader(1);
% Vortex age tau:
tau = norm(dx / norm(leader.vGround_NED_m));
vortexStrength = 0.5 * leader.CL * norm(leader.vGround_NED_m) * leader.cbar;
% TODO Have a closer look at the empirical value for nu used in eq. 32.
empiricalNu = 0.06*vortexStrength;
rc = 2.24 * sqrt(empiricalNu * tau);
% Compute the radial distance between the filament and the
% point, i.e. the distance in the wind frame's yz plane:
dfp = p_wleader - p0_filament_wleader;
rr = norm(dfp(2:3));
vi_wleader = (vortexStrength/(2*pi*rr)) * (1 - exp(-1.26 * (rr/rc)^2));
% Get unit vector pointing in the direction of the induced
% flow:
v_radius_wleader = [0; dfp(2:3, :)];
u_flow_wleader = fflib_normalize(cross(u_senseofrotation_wleader, v_radius_wleader));
vi_wleader = vi_wleader * u_flow_wleader;
end

% Transforms a given position from the body frame to the NED frame.
function p_NED = body2NED(p_b, pBodyframe_NED, qAttitude_NED)
p_NED = fflib_frame2frame(p_b, pBodyframe_NED, qAttitude_NED);
end

% Finds those UASs whose wake vortices have a relevant impact on this
% one.
function indices = selectDominantUASs(ID)
% FIXME Do proper selection
if ID == 3
    indices = [1 2];
elseif ID == 2
    indices = 1;
else
    indices = [];
end
end

