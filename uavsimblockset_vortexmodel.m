% Based on "Modeling of Aerodynamic Coupling Between Aircraft in Close
% Proximity", Dogan et al.
function [Vi_bfrozen, omegai_bfrozen, Vi_NED, p0_filament_left, p0_filament_right, dcm_ew, debugport] = uavsimblockset_vortexmodel(...
    ID, ...
    tsim, ...
    VortexSig)

debugport = zeros(10,1);
stack.vi_center_NED = [0 0 0]';
stack.vi_eff_b = [0 0 0]';
stack.omegai_eff_b = [0 0 0]';
stack.checkpoints.center = [0 0 0]';
stack.checkpoints.nose = zeros(3,1);
stack.checkpoints.rear = zeros(3,1);
stack.checkpoints.leftwingtip = zeros(3,1);
stack.checkpoints.rightwingtip = zeros(3,1);
stack.checkpoints.fintip = zeros(3,1);
stack.checkpoints.finbottom = zeros(3,1);
% Compute unit vectors pointing along the wing:
stack.leftwingvector = zeros(3,1);
stack.rightwingvector = zeros(3,1);

%test = linspace(2,3,randi(100));

% Inialize shared values structure:
stack.DCM_bw_leader = eye(3);
stack.DCM_eb_leader = eye(3);
stack.DCM_ew_leader = eye(3);
stack.DCM_bw_this = eye(3);
stack.DCM_eb_this = eye(3);
stack.DCM_ew_this = eye(3);

stack.DCM_be_follower = eye(3);
stack.DCM_windLeader2bodyFollower = eye(3);
stack.p0_rightFilament_wleader = zeros(3,1);
stack.p0_leftFilament_wleader = zeros(3,1);
stack.p0_rightFilament_bleader = zeros(3,1);
stack.p0_leftFilament_bleader = zeros(3,1);

% How many samples to use for averaging:
stack.n_averagesamples = 10;
stack.winglength = 0;

% Compute necessary rotation matrices:
this = VortexSig(ID);
stack.DCM_bw_this = ffb_dcmbody2wind(this.alpha_rad, this.beta_rad)';
stack.DCM_eb_this = ffb_quat2dcm(this.qAttitude')';
stack.DCM_ew_this = stack.DCM_eb_this * stack.DCM_bw_this;

% Select VortexSig that have a relevant impact on this one:
[relevantUAS, n_relevantUAS] = selectDominantUASs(ID, VortexSig, stack);
debugport(1) = n_relevantUAS;

if n_relevantUAS > 0
    relevantUAS = relevantUAS(1:n_relevantUAS);
    for j=1:length(relevantUAS)
        % UAS inducing flow:
        maximoleader = VortexSig(relevantUAS(j));
        if ~isempty(maximoleader)
            % Computation fails for two vehicles that are exactly in the same
            % place, as might be case at the very beginning of a simulation:
            % if norm(maximoleader.p_NED_m - VortexSig(ID).p_NED_m) > 1e-3
            % FIXME: Not reliable, just wait a little as workaround:
            if tsim > 0.1;
                % Compute rotation matrix from the leader's wind frame to the
                % follower's body frame:
                % DCM (wind leader) -> (body leader)
                stack.DCM_bw_leader = ffb_dcmbody2wind(maximoleader.alpha_rad, maximoleader.beta_rad)';
                % DCM (body leader) -> NED
                stack.DCM_eb_leader = ffb_quat2dcm(maximoleader.qAttitude')';
                % DCM (wind leader) -> NED
                stack.DCM_ew_leader = stack.DCM_eb_leader * stack.DCM_bw_leader;
                % DCM NED -> (body follower)
                stack.DCM_be_follower = ffb_quat2dcm(VortexSig(ID).qAttitude');
                stack.DCM_windLeader2bodyFollower = stack.DCM_be_follower * stack.DCM_ew_leader;
                % Compute source points of vortex filaments:
                % unit vector that points from the wind frame's origin to the right
                % wing tip:
                stack.p0_rightFilament_bleader = getWingPoint(maximoleader,  0.5 * maximoleader.b_m * pi/4, 1);
                stack.p0_rightFilament_wleader = stack.DCM_bw_leader' * stack.p0_rightFilament_bleader;
                % The same procedure for the left wing:
                stack.p0_leftFilament_bleader = getWingPoint(maximoleader,  0.5 * maximoleader.b_m * pi/4, -1);
                stack.p0_leftFilament_wleader = stack.DCM_bw_leader' * stack.p0_leftFilament_bleader;
                
                % Compute bounding points:
                if ~isfield(VortexSig(ID), 'checkpoints')
                    stack.rightwingvector = ffb_angle2dcm(this.sweep_rad, this.dihedral_rad, 0)' * [0 1 0]';
                    stack.winglength = this.b_m/stack.rightwingvector(2);
                    stack.checkpoints.center = [0 0 0]';
                    stack.checkpoints.nose = VortexSig(ID).pNose_b_m;
                    stack.checkpoints.rear = VortexSig(ID).pRear_b_m;
                    stack.checkpoints.leftwingtip = getWingPoint(VortexSig(ID),  0.5 * maximoleader.b_m, -1);
                    stack.checkpoints.rightwingtip = getWingPoint(VortexSig(ID),  0.5 * maximoleader.b_m, 1);
                    stack.checkpoints.fintip = VortexSig(ID).posFinTip_b_m;
                    stack.checkpoints.finbottom = VortexSig(ID).posFuselageBottom_b_m;
                    % Compute unit vectors pointing along the wing:
                    stack.leftwingvector = fflib_normalize(stack.checkpoints.leftwingtip);
                    stack.rightwingvector = fflib_normalize(stack.checkpoints.rightwingtip);
                    VortexSig(ID).checkpoints = stack.checkpoints;
                end
                [test, stack.vi_center_NED]= getVi(stack.checkpoints.center, ...
                    VortexSig(ID), maximoleader, ...
                    stack);
                
                debugos = true;
                if debugos
                    computeDistributions(ID, VortexSig, stack, maximoleader);
                end
                [stack] = computeEffectiveVAndOmega(VortexSig, ID, maximoleader, stack);
            end
        end
    end
end

Vi_bfrozen = stack.vi_eff_b;
omegai_bfrozen = stack.omegai_eff_b;
Vi_NED = stack.vi_center_NED;

% Compute origin and center axis of vortices for this UAS:
% Compute vectors in NED:
p0_rightFilament_b = getWingPoint(VortexSig(ID),  0.5 * VortexSig(ID).b_m * pi/4, 1);
p0_rightFilament_NED = body2NED(p0_rightFilament_b, VortexSig(ID).p_NED_m, VortexSig(ID).qAttitude);
p0_leftFilament_b = getWingPoint(VortexSig(ID),  0.5 * VortexSig(ID).b_m * pi/4, -1);
p0_leftFilament_NED = body2NED(p0_leftFilament_b, VortexSig(ID).p_NED_m, VortexSig(ID).qAttitude);

p0_filament_right = p0_rightFilament_NED;
p0_filament_left = p0_leftFilament_NED;
dcm_ew = stack.DCM_ew_this;
if tsim > 1 && ID == 3
    bp = 0;
end
end

% Computes the average induced velocity and �nduced body rotation rates:
function stack_out = computeEffectiveVAndOmega(VortexSig, ID, maximoleader, stack_in)
stack_out = stack_in;
this = VortexSig(ID);
% Best fit approach from the article:
% Weighting functions normalized for a range of 0 to 1:
ds = ffb_linspace(0,1,stack_out.n_averagesamples);
Lf = norm(this.pRear_b_m - this.pNose_b_m);
Df = norm(this.posFuselageBottom_b_m - this.posFinTip_b_m);
b = this.b_m;
fxy = zeros(size(ds));
fzy = fxy;
fxz = fxy;
for k=1:stack_out.n_averagesamples
    fxy(k) = ds(k)*norm(cos(this.dihedral_rad)*cos(this.sweep_rad))/(3*Lf/4);
    fzy(k) = ds(k)*norm(cos(this.dihedral_rad)*cos(this.sweep_rad))/(b/2);
    fxz(k) = ds(k)/(3*Lf/4);
    fxz(k) = ds(k)/(b/2);
    fz1x =  fxz;
    fy1x =  fxz;
    fz2x =  fxz;
    fy2x =  fxz;
end
Wx1y = (b/2)*weightedaverage([0 -b/2 0]', [0 0 0]', VortexSig, ID, maximoleader, stack_in,  fxy ,1);
Wx2y = (b/2)*weightedaverage([0 0 0]', [0 b/2 0]', VortexSig, ID, maximoleader, stack_in,  fxy, 1);
Wx1z = (Df/2)*weightedaverage([0 0 -Df/2]', [0 0 0]', VortexSig, ID, maximoleader, stack_in,  fxz, 1);
Wx2z = (Df/2)*weightedaverage([0 0 0]', [0 0 Df/2]', VortexSig, ID, maximoleader, stack_in,  fxz, 1);
% Ok, this is too fucked up, dropping this shit. Comparison can be done
% anyway.

vi_center2nose = average(stack_in.checkpoints.center, stack_in.checkpoints.nose, VortexSig, ID, maximoleader, stack_in);
vi_center2rear = average(stack_in.checkpoints.center, stack_in.checkpoints.rear, VortexSig, ID, maximoleader, stack_in);
vi_center2rightwingtip = average(stack_in.checkpoints.center, stack_in.checkpoints.rightwingtip, VortexSig, ID, maximoleader, stack_in);
vi_center2leftwingtip = average(stack_in.checkpoints.center, stack_in.checkpoints.leftwingtip, VortexSig, ID, maximoleader, stack_in);
vi_center2fintip = average(stack_in.checkpoints.center, stack_in.checkpoints.fintip, VortexSig, ID, maximoleader, stack_in);
vi_center2finbottom = average(stack_in.checkpoints.center, stack_in.checkpoints.finbottom, VortexSig, ID, maximoleader, stack_in);
Vi_averaged = 1/4 * (vi_center2nose ...
    + vi_center2rear ...
    + vi_center2fintip ...
    + vi_center2finbottom);

% Least squares approach: fit angular rates and average induced velocity to
% pointwise induced velocity vectors
vi_alongBodyX = distribution(VortexSig(ID), [0 0 0]', [1 0 0]', [-Lf/2, Lf/2], stack_in, maximoleader);
vi_alongBodyY = distribution(VortexSig(ID),[0 -b/2 0]', [0 1 0]' , [0 b], stack_in, maximoleader);
vi_alongBodyZ = distribution(VortexSig(ID),[0 0 0]', [0 0 1]', [-Df/2, Df/2]', stack_in, maximoleader);
% Start with average induced velocity:
data = [vi_alongBodyX vi_alongBodyY vi_alongBodyZ];
y = data(2:4, :);
y = reshape(y, [], 1);
A = repmat(eye(3), length(y)/3, 1);
Vi_lsq = (A'*A)^-1*A'*y;
% Now angular rates:
% The A matrix now indicates how angular rates generate induced velocities:
ps = data(5:7, :);
for k=1:size(ps, 2);
    p = ps(:, k);
    rw = 1+(k-1)*3;
    A(rw:rw+2, :) = [0 -p(3) p(2); 
                       p(3) 0 -p(1);
                       -p(2) p(1) 0];
end
omegai_lsq = (A'*A)^-1*A'*y;

%stack_out.vi_eff_b = Vi_averaged;
stack_out.vi_eff_b = Vi_lsq;
stack_out.omegai_eff_b = omegai_lsq;
end


% Computes the induced velocity distribution over one airframe
% segment defined by a point in the body frame, a unit vector and
% a range.
function vi_dist= distribution(inducee, p0, u, r, stack, inducer)
% Generate evenly spaced points:
div = 100;
steps = zeros(div+1, 1);
steps = ffb_linspace(r(1),r(2), div);
vi_dist = zeros(7,length(steps));
for i=1:length(steps);
    p = p0 + steps(i) * u;
    [vi_dist(2:4, i), ~] = getVi(p, inducee, inducer, stack);
    vi_dist(5:7, i) = p;
    vi_dist(1, i) = steps(i);
end
end

% Computes the average induced velocity over one airframe segment
% by integrating between two checkpoints in the body frame.
function vi_av_ax = weightedaverage(p1, p2, VortexSig, ID, maximoleader, stack, f, axis)
% Generate evenly spaced integration points:
l = norm(p1-p2);
steps = zeros(stack.n_averagesamples,1);
div = length(steps);
for k=2:div
    steps(k) = steps(k-1) + (l/div);
end
unit_p1top2 = fflib_normalize(p2-p1);
vi_av = zeros(3,1);
for i=1:length(steps);
    p = p1 + steps(i) * unit_p1top2;
    [vi_atPoint_b, ~] = getVi(p, VortexSig(ID), maximoleader, stack);
    vi_av = vi_av + vi_atPoint_b*(1+steps(i)*f(i));
end
vi_av = vi_av ./ length(steps);
vi_av_ax = vi_av(axis);
end

% Computes the average induced velocity over one airframe segment
% by integrating between two checkpoints in the body frame.
function vi_av = average(p1, p2, VortexSig, ID, maximoleader, stack)
% Generate evenly spaced integration points:
l = norm(p1-p2);
steps = zeros(stack.n_averagesamples,1);
div = length(steps);
for k=2:div
    steps(k) = steps(k-1) + (l/div);
end
unit_p1top2 = fflib_normalize(p2-p1);
vi_av = zeros(3,1);
for i=1:length(steps);
    p = p1 + steps(i) * unit_p1top2;
    [vi_atPoint_b, ~] = getVi(p, VortexSig(ID), maximoleader, stack);
    vi_av = vi_av + vi_atPoint_b;
end
vi_av = vi_av ./ length(steps);
end

% Computes the induced velocity distribution at discrete points:
function computeDistributions(ID, VortexSig, stack, maximoleader)
b = VortexSig(ID).b_m;

vi_alongBodyX = distribution(VortexSig(ID), [0 0 0]', [1 0 0]', [-60*b, 20*b], stack, maximoleader);
vi_alongBodyZ = distribution(VortexSig(ID),[0 0 0]', [0 0 1]', [-2*b, 2*b]', stack, maximoleader);
vi_alongtheleftwing = distribution(VortexSig(ID),[0 0 0]', stack.leftwingvector, [0 3*b], stack, maximoleader);
vi_alongtheleftwing(1,:) = -vi_alongtheleftwing(1,:);
vi_alongtherightwing = distribution(VortexSig(ID),[0 0 0]', stack.rightwingvector, [0 3*b], stack, maximoleader);
vi_alongthewing = [vi_alongtheleftwing vi_alongtherightwing];
%
% % Piotr:
figure(84)
subplot(3,2,1)
bref = maximoleader.b_m;
plot(vi_alongthewing(1,:)./bref, vi_alongthewing(2,:) );
title('Wx along the wing')
grid on;
axis([-4,4,-0.4,0.3]);
subplot(3,2,2)
plot(vi_alongBodyZ(1,:)./bref, vi_alongBodyZ(2,:) );
title('Wx along body z')
axis([-2,2,-0.01,0.06]);
grid on;

subplot(3,2,3)
plot(vi_alongBodyX(1,:)./bref, vi_alongBodyX(3,:) );
title('Wy along body x')
axis([-60,20,-0.2,0.3]);
grid on;
subplot(3,2,4)
plot(vi_alongBodyZ(1,:)./bref, vi_alongBodyZ(3,:) );
title('Wy along body z')
axis([-2,2,-0.4,0.3]);
grid on;

subplot(3,2,5)
plot(vi_alongBodyX(1,:)./bref, vi_alongBodyX(4,:) );
title('Wz along body x')
axis([-60,20,-0.4,0.1]);
grid on;
subplot(3,2,6)
plot(vi_alongthewing(1,:)./bref, vi_alongthewing(4,:) );
title('Wz along the wing')
axis([-4,4,-1.5,2.5]);
grid on;

drawnow;
end

% Generates an array of evenly spaced points between two given scalars a
% and b, for which b > a.
function lspace = ffb_linspace(a, b, n)
dl = (b-a)/n;
lspace = zeros(n+1, 1);
lspace(1) = a;
for k=2:length(lspace)
    lspace(k) = lspace(k-1) + dl;
end

end

% Computes the body frame coordinates of a point on the wing of the given
% UAS for the given distance from the body frame's x-z plane. The third
% parameter indicates the halfplane the point is supposed to lie in
% (1=right wing, -1=left wing). Helpful e.g. to compute the positions of
% the wing tips for an aircraft featuring dihedral and sweep angle.
function p = getWingPoint(ac, dy, hlfpl)
% Compute unit vector pointing from the origin to the wing tip:
wingvector = ffb_angle2dcm((hlfpl)*ac.sweep_rad, (hlfpl)*ac.dihedral_rad, 0)' * [0 hlfpl 0]';
winglength = abs(ac.b_m/wingvector(2));
p = wingvector* dy*(winglength/ac.b_m);
end

% Computes the induced velocity vector in the follower's body frame at a
% given point defined in the follower's body frame.
function [vi_b, vi_NED]= getVi(p_bfollower, follower, leader, stack)
% Find point coordinates in the leader's wind frame:
% First compute position of the follower's body frame in the
% leader's wind frame.
% Relative position vector between leader the point of interest in the NED
% frame:
d_NED =  body2NED(p_bfollower, follower.p_NED_m, follower.qAttitude) - leader.p_NED_m;
% Rotate to the leader's wind frame:
p_wleader = stack.DCM_ew_leader' * d_NED;
% Compute the velocities induced by the right and the left
% filament:
% Get induced velocities:
vi_wleader = computeVi(p_wleader, stack.p0_leftFilament_wleader, [1 0 0]', leader) ...
    + computeVi(p_wleader, stack.p0_rightFilament_wleader, [-1 0 0]', leader);
vi_b = stack.DCM_windLeader2bodyFollower * vi_wleader;
vi_NED = stack.DCM_be_follower' * vi_b;
end

% Computes the induced velocity vector for a given filament that
% starts at the given point in the leader's wind frame.
% The unit vector u_senseofrotation_wleader indicates the sense of
% rotation of the vortex (right hand rule).
function vi_wleader = computeVi(p_wleader, p0_filament_wleader, u_senseofrotation_wleader, leader)
% To compute the norm of the induced velocity, we need the
% longitudinal separation to compute the vortex age tau:
dx = abs(p_wleader(1));
% Vortex age tau:
tau = norm(dx / norm(leader.v_NED_mps));
% Disable core:
% tau = 1e-6;
vortexStrength = 0.5 * leader.CL * norm(leader.v_NED_mps) * leader.cbar_m;
empiricalNu = 0.06*vortexStrength;
rc = 2.24 * sqrt(empiricalNu * tau);
% Compute the radial distance between the filament and the
% point, i.e. the distance in the wind frame's yz plane:
dfp = p_wleader - p0_filament_wleader;
rr = norm(dfp(2:3));
%vi_wleader = (vortexStrength/(2*pi*rr)) * (1 - exp(-1.26 * (rr/rc)^2));
vi_wleader = (vortexStrength/(2*pi)) * rr/(rr^2 + rc^2);
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
function [indices, n] = selectDominantUASs(ID, VortexSig, stack)
% Figure out how many active UAS there are by finding zero fields in the
% shared state:
n = 0;
indices = zeros(10,1);
for k=1:length(VortexSig)
    if (VortexSig(k).timeOfLastUpdate > eps) && (k ~= ID)
        % Check whether the UAS is in front of this one:
        dp_e = VortexSig(k).p_NED_m - VortexSig(ID).p_NED_m;
        % rotate to this UAS's wind frame:
        dp_w = stack.DCM_ew_this' * dp_e;
        if dp_w(1) > 0
            n = n + 1;
            indices(n) = k;
        end
    end
    
end
if n < 1 && ID == 3
    bp = 0;
end
end