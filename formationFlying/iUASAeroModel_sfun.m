% This Level-2 S-function provides functionality to compute approximate
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
function iUASAeroModel_sfun( block )
setup(block);
end

function setup(block)

% Register number of ports
% Input ports:
% - position vector (NED)
% - velocity vector (NED)
% - lift coefficient CL
% - attitude quaternion
% - wind angles alpha and beta
block.NumInputPorts  = 5;
% Output ports:
% - induced velocity in the bfrozen frame
% - induced angular rate in the bfrozen frame
% - induced velocity vector at the body frame's origin given in the NED frame
block.NumOutputPorts = 3;
% Block parameters:
%   - aircraft ID
%   - mean aerodynamic chord c
%   - wing span
%   - sweep angle
%   - dihedral angle
%   - longitudinal reference length
%   - nose coordinates (body frame)
%   - vertical reference length
%   - fin tip coordinates (body frame)
block.NumDialogPrms = 9;

% Set input port 1 properties
block.InputPort(1).SamplingMode = 'sample';
block.InputPort(1).Dimensions  = 3;
block.InputPort(1).DatatypeID  = 0; % double
block.InputPort(1).Complexity  = 'Real';
% Set input port 2 properties
block.InputPort(2).SamplingMode = 'sample';
block.InputPort(2).Dimensions  = 3;
block.InputPort(2).DatatypeID  = 0; % double
block.InputPort(2).Complexity  = 'Real';
% Set input port 3 properties
block.InputPort(3).SamplingMode = 'sample';
block.InputPort(3).Dimensions  = 1;
block.InputPort(3).DatatypeID  = 0; % double
block.InputPort(3).Complexity  = 'Real';
% Set input port 4 properties: attitude quaternion
block.InputPort(4).SamplingMode = 'sample';
block.InputPort(4).Dimensions  = 4;
block.InputPort(4).DatatypeID  = 0; % double
block.InputPort(4).Complexity  = 'Real';
% Set input port 5 properties: wind angles
block.InputPort(5).SamplingMode = 'sample';
block.InputPort(5).Dimensions  = 2;
block.InputPort(5).DatatypeID  = 0; % double
block.InputPort(5).Complexity  = 'Real';

% Set output port 1 properties
block.OutputPort(1).SamplingMode = 'sample';
block.OutputPort(1).Dimensions  = 3;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
% Set output port 2 properties
block.OutputPort(2).SamplingMode = 'sample';
block.OutputPort(2).Dimensions  = 3;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
% Set output port 3 properties
block.OutputPort(3).SamplingMode = 'sample';
block.OutputPort(3).Dimensions  = 3;
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Terminate', @Terminate); % Required
block.RegBlockMethod('PostPropagationSetup', @DoPostPropagationSetup );
block.RegBlockMethod('InitializeConditions', @DoStateInit );
block.RegBlockMethod('Update', @DoUpdate );
block.RegBlockMethod('Start', @DoStart);
end


function DoStart(block)
% Delete static/persistent variables of the DoUpdate function to make it
% ready for this simulation run, which may for instance feature a different
% number of UASs.
Vars=whos;
PersistentVars=Vars([Vars.persistent]);
PersistentVarNames={PersistentVars.name};
clear(PersistentVarNames{:});
clear DoUpdate;
clear UASs;
end

% Computes the induced flow acting on the UAs connected to the block
% calling this function. To this end, this function uses a list of all UASs
% in the airspace and computes the combined flow induced by those that are
% close enough to have a relevant impact.
function DoUpdate(block)
% Indicates whether certain debug variables such as the induced velocity
% distributions are to be computed. Set to 'false' for simulation to save
% on execution time.
debug = false;

% Set outputs for the case that there are no flow-inducing UASs around:
block.OutputPort(1).Data = zeros(3,1);
block.OutputPort(2).Data = zeros(3,1);
block.OutputPort(3).Data = zeros(3,1);

% This cell array holds position, velocity etc. of all UAS sending data to
% this s-function. It serves as a data base / register to compute the
% induced flow acting on each of them.
persistent UASs;

% Get ID of the calling UAS and put its data in the UAS register:
ID = block.DialogPrm(1).Data;
UASs{ID}.ID = ID;
UASs{ID}.cbar = block.DialogPrm(2).Data;
UASs{ID}.b = block.DialogPrm(3).Data;
UASs{ID}.sweep_rad = block.DialogPrm(4).Data;
UASs{ID}.dihedral_rad = block.DialogPrm(5).Data;
UASs{ID}.refLongi = block.DialogPrm(6).Data;
UASs{ID}.nose_b_m = block.DialogPrm(7).Data';
UASs{ID}.refVertical = block.DialogPrm(8).Data;
UASs{ID}.fintip_b_m = block.DialogPrm(9).Data';

UASs{ID}.p_NED_m = block.InputPort(1).Data;
UASs{ID}.vGround_NED_m = block.InputPort(2).Data;
UASs{ID}.CL = block.InputPort(3).Data;
UASs{ID}.qAttitude = block.InputPort(4).Data;
UASs{ID}.alpha = block.InputPort(5).Data(1);
UASs{ID}.beta = block.InputPort(5).Data(2);
UASs{ID}.vi_center_NED = [0 0 0]';
UASs{ID}.vi_eff_b = [0 0 0]';
UASs{ID}.omegai_eff_b = [0 0 0]';

% Select UASs that have a relevant impact on this one:
relevantUASs = selectDominantUASs();
for j=1:length(relevantUASs)
    % UAS inducing flow:
    maximoleader = UASs{relevantUASs(j)};
    if ~isempty(maximoleader)
        % Computation fails for two vehicles that are exactly in the same
        % place, as might be case at the very beginning of a simulation:
        if norm(maximoleader.p_NED_m - UASs{ID}.p_NED_m) > eps
            % Compute rotation matrix from the leader's wind frame to the
            % follower's body frame:
            % DCM (wind leader) -> (body leader)
            DCM_bw_leader = dcmbody2wind(maximoleader.alpha, maximoleader.beta)';
            % DCM (body leader) -> NED
            DCM_eb_leader = quat2dcm(maximoleader.qAttitude')';
            % DCM (wind leader) -> NED
            DCM_ew_leader = DCM_eb_leader * DCM_bw_leader;
            % DCM NED -> (body follower)
            DCM_be_follower = quat2dcm(UASs{ID}.qAttitude');
            DCM_windLeader2bodyFollower = DCM_be_follower * DCM_ew_leader;
            % Compute source points of vortex filaments:
            % unit vector that points from the wind frame's origin to the right
            % wing tip:
            % TODO Handling of b not entirely correct.
            p0_rightFilament_bleader = getWingPoint(maximoleader,  0.5 * maximoleader.b * pi/4, 1);
            p0_rightFilament_wleader = DCM_bw_leader' * p0_rightFilament_bleader;
            % The same procedure for the left wing:
            p0_leftFilament_bleader = getWingPoint(maximoleader,  0.5 * maximoleader.b * pi/4, -1);
            p0_leftFilament_wleader = DCM_bw_leader' * p0_leftFilament_bleader;
            
            % Compute bounding points:
            if ~isfield(UASs{ID}, 'checkpoints')
                UASs{ID}.checkpoints.center = [0 0 0]';
                UASs{ID}.checkpoints.nose = UASs{ID}.nose_b_m;
                UASs{ID}.checkpoints.rear = UASs{ID}.nose_b_m + [-UASs{ID}.refLongi 0 0]';
                UASs{ID}.checkpoints.leftwingtip = getWingPoint(UASs{ID},  0.5 * maximoleader.b, -1);
                UASs{ID}.checkpoints.rightwingtip = getWingPoint(UASs{ID},  0.5 * maximoleader.b, 1);
                UASs{ID}.checkpoints.fintip = UASs{ID}.fintip_b_m;
                UASs{ID}.checkpoints.finbottom = UASs{ID}.fintip_b_m + [0 0 -UASs{ID}.refVertical]';
                % Compute unit vectors pointing along the wing:
                UASs{ID}.leftwingvector = fflib_normalize(UASs{ID}.checkpoints.leftwingtip);
                UASs{ID}.rightwingvector = fflib_normalize(UASs{ID}.checkpoints.rightwingtip);
            end
            [test, UASs{ID}.vi_center_NED]= getVi(UASs{ID}.checkpoints.center, UASs{ID}, maximoleader);
            
            if debug
                computeInducedV();
            end
            computeEffectiveVAndOmega();
        end
    end
end

block.OutputPort(1).Data = UASs{ID}.vi_eff_b;
block.OutputPort(2).Data = UASs{ID}.omegai_eff_b;
block.OutputPort(3).Data = UASs{ID}.vi_center_NED;

if block.CurrentTime < 1
    block.OutputPort(1).Data = zeros(3,1);
    block.OutputPort(2).Data = zeros(3,1);
    block.OutputPort(3).Data = zeros(3,1);
end

% Computes the average induced velocity and índuced body rotation rates:
    function computeEffectiveVAndOmega()
        vi_center2nose = average(UASs{ID}.checkpoints.center, UASs{ID}.checkpoints.nose);
        vi_center2rear = average(UASs{ID}.checkpoints.center, UASs{ID}.checkpoints.rear);
        vi_center2rightwingtip = average(UASs{ID}.checkpoints.center, UASs{ID}.checkpoints.rightwingtip);
        vi_center2leftwingtip = average(UASs{ID}.checkpoints.center, UASs{ID}.checkpoints.leftwingtip);
        vi_center2fintip = average(UASs{ID}.checkpoints.center, UASs{ID}.checkpoints.fintip);
        vi_center2finbottom = average(UASs{ID}.checkpoints.center, UASs{ID}.checkpoints.finbottom);
        
        UASs{ID}.vi_eff_b(1) = 1/4 * (vi_center2rightwingtip(1) ...
            + vi_center2leftwingtip(1) ...
            + vi_center2fintip(1) ...
            + vi_center2finbottom(1));
        
        UASs{ID}.vi_eff_b(2) = 1/4 * (vi_center2nose(2) ...
            + vi_center2rear(2) ...
            + vi_center2fintip(2) ...
            + vi_center2finbottom(2));
        
        UASs{ID}.vi_eff_b(3) = 1/4 * (vi_center2rear(3) ...
            + vi_center2nose(3) ...
            + vi_center2rightwingtip(3) ...
            + vi_center2leftwingtip(3));
        
        % Computes the average induced velocity over one airframe segment
        % by integrating between two checkpoints in the body frame.
        function vi_av = average(p1, p2)
            % Generate evenly spaced integration points:
            ds = 0.1;
            l = norm(p1-p2);
            steps = 0:ds:l;
            unit_p1top2 = fflib_normalize(p2-p1);
            vi_av = zeros(3,1);
            for i=1:length(steps);
                p = p1 + steps(i) * unit_p1top2;
                [vi_atPoint_b, ~] = getVi(p, UASs{ID}, maximoleader);
                vi_av = vi_av + vi_atPoint_b;
            end
            vi_av = vi_av ./ length(steps);
        end
    end


% Computes the induced velocity distribution at discrete points:
    function computeInducedV()
        b = UASs{ID}.b;
        
        vi_alongBodyX = distribution([0 0 0]', [1 0 0]', [-60*b, 20*b]);
        vi_alongBodyZ = distribution([0 0 0]', [0 0 1]', [-2*b 2*b]');
        vi_alongtheleftwing = distribution([0 0 0]', UASs{ID}.leftwingvector, [0 4*b]);
        vi_alongtherightwing = distribution([0 0 0]', - UASs{ID}.rightwingvector, [0 -4*b]);
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
        
        % Computes the induced velocity distribution over one airframe
        % segment defined by a point in the body frame, a unit vector and
        % a range.
        function vi_dist = distribution(p0, u, r)
            % Generate evenly spaced points:
            ds = UASs{ID}.b/20;
            steps = linspace(r(1),r(2), ceil(abs(r(2)-r(1))/ds));
            vi_dist = zeros(4,length(steps));
            ps = [];
            for i=1:length(steps);
                p = p0 + steps(i) * u;
                [vi_dist(2:4, i), ~] = getVi(p, UASs{ID}, maximoleader);
                ps = [ps p];
                vi_dist(1, i) = steps(i)/UASs{ID}.b;
            end
        end
    end

% Computes the body frame coordinates of a point on the wing of the given
% UAS for the given distance from the body frame's origin. The third
% parameter indicates the halfplane the point is supposed to lie in
% (1=right wing, -1=left wing). Helpful e.g. to compute the positions of
% the wing tips for an aicraft featuring dihedral and sweep angle.
    function p = getWingPoint(ac, r, hlfpl)
        % Compute unit vector pointing from the origin to the wing tip:
        wingvector = angle2dcm((hlfpl)*ac.sweep_rad, (hlfpl)*ac.dihedral_rad, 0, 'ZXY')' * [0 hlfpl 0]';
        p = wingvector * r;
    end

% Computes the induced velocity vector in the follower's body frame at a
% given point defined in the follower's body frame.
    function [vi_b, vi_NED]= getVi(p_bfollower, follower, leader)
        % Find point coordinates in the leader's wind frame:
        % First compute position of the follower's body frame in the
        % leader's wind frame.
        % relative position vector in the NED frame:
        d_NED = body2NED(p_bfollower, follower.p_NED_m, follower.qAttitude) - leader.p_NED_m;
        % rotate to the leader's wind frame:
        d_wleader = DCM_ew_leader' * d_NED;
        % Transform point:
        p_wleader = fflib_frame2frame(p_bfollower, d_wleader, dcm2quat(DCM_windLeader2bodyFollower)');
        % Compute the velocities induced by the right and the left
        % filament:
        % Get induced velocities:
        vi_wleader = computeVi(p_wleader, p0_leftFilament_wleader, [1 0 0]') ...
            + computeVi(p_wleader, p0_rightFilament_wleader, [-1 0 0]');
        vi_b = DCM_windLeader2bodyFollower * vi_wleader;
        vi_NED = DCM_be_follower' * vi_b;
        
        % Computes the induced velocity vector for a given filament that
        % starts at the given point in the leader's wind frame.
        % The unit vector u_senseofrotation_wleader indicates the sense of
        % rotation of the vortex (right hand rule).
        function vi_wleader = computeVi(p_wleader, p0_filament_wleader, u_senseofrotation_wleader)
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
        
    end

% Transforms a given position from the body frame to the NED frame.
    function p_NED = body2NED(p_b, pBodyframe_NED, qAttitude_NED)
        p_NED = fflib_frame2frame(p_b, pBodyframe_NED, qAttitude_NED);
    end

% Finds those UASs whose wake vortices have a relevant impact on this
% one.
    function indices = selectDominantUASs()
        % FIXME Do proper selection
        if ID == 3
            indices = [1 2];
        elseif ID == 2
            indices = 1;
        else
            indices = [];
        end
    end

end

function DoPostPropagationSetup(block)
end

function DoStateInit(block)
end

function Outputs(block)

end %function



function Terminate(block)

end