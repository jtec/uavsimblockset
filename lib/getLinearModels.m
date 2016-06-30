%
% Builds linear models for controller design.
%
function linear = getLinearModels(trimmod, xTrim, uTrim, yTrim)

% Get linear open loop model excluding actuators:
% Indices of output ports that are selected as new states:
ix = [1 2 3:5 7:12 15]; % alpha

[linear.eulers.ss_full, linear.eulers.trim, linear.eulers.imap] = uavsimblockset_linearize(trimmod, ...
    xTrim, uTrim, yTrim, ...
    ix);

% Map that links state/output/input names to indices:
imap = linear.eulers.imap;
linear.operatingQBar = 0.5 * 1.225 * xTrim(imap.x('Va_mps'))^2;
% Extract submodels:
longistates = [imap.x('Va_mps') imap.x('alpha_rad') imap.x('theta_rad') imap.x('q_rps') imap.x('x_NED_m') imap.x('z_NED_m')];
longiinputs = [imap.u('de_rad') imap.u('den_0to1') imap.u('vw_x_mps') imap.u('vw_z_mps') imap.u('omegaw_y_rps')];
longioutputs = [imap.y('Va_mps') imap.y('alpha_rad') imap.y('q_rps') imap.y('alpha_rad') imap.y('x_NED_m') imap.y('z_NED_m')];
linear.eulers.ss_longi = getSubsystem(linear.eulers.ss_full, ...
    longistates, longiinputs, longioutputs);
linear.eulers.ss_longi_withoutwind = getSubsystem(linear.eulers.ss_full, ...
    longistates, [imap.u('de_rad') imap.u('den_0to1')], longioutputs);

longistates = [imap.x('Va_mps') imap.x('alpha_rad') imap.x('theta_rad') imap.x('q_rps')];
longiinputs = [imap.u('de_rad') imap.u('den_0to1')];
longioutputs = [imap.y('Va_mps') imap.y('alpha_rad') imap.y('theta_rad') imap.y('q_rps')];
linear.eulers.ss_longi_speed_withoutwind = getSubsystem(linear.eulers.ss_full, ...
    longistates, longiinputs, longioutputs);

lateralstates = [imap.x('beta_rad') imap.x('phi_rad') imap.x('vy_NED_mps') imap.x('p_rps') imap.x('r_rps') imap.x('y_NED_m')];
lateralinputs = [imap.u('da_rad') imap.u('dr_rad') imap.u('vw_y_mps') imap.u('omegaw_x_rps') imap.u('omegaw_z_rps')];
lateraloutputs = [imap.y('beta_rad') imap.y('phi_rad') imap.y('vy_NED_mps') imap.y('p_rps') imap.y('r_rps') imap.y('y_NED_m')];
linear.eulers.ss_lateral = getSubsystem(linear.eulers.ss_full, ...
    lateralstates, lateralinputs, lateraloutputs);

x = [imap.x('beta_rad') imap.x('phi_rad') imap.x('vy_NED_mps') imap.x('p_rps') imap.x('r_rps') imap.x('y_NED_m')];
u = [imap.u('da_rad') imap.u('dr_rad')];
y = [imap.y('beta_rad') imap.y('phi_rad') imap.y('vy_NED_mps') imap.y('p_rps') imap.y('r_rps') imap.y('y_NED_m')];
linear.eulers.ss_lateral_withoutwind = getSubsystem(linear.eulers.ss_full, x, u, y);

states = [imap.x('beta_rad') imap.x('phi_rad') imap.x('p_rps') imap.x('r_rps') ];
inputs = [imap.u('da_rad') imap.u('dr_rad')];
outputs = [imap.y('beta_rad') imap.y('phi_rad') imap.y('p_rps') imap.y('r_rps')];
linear.eulers.ss_lateral_rotatory_withoutWind = getSubsystem(linear.eulers.ss_full, ...
    states, inputs, outputs);

shortperiodstates = [imap.x('alpha_rad') imap.x('q_rps')];
shortperiodinputs = [imap.u('de_rad')];
shortperiodoutputs = [imap.y('alpha_rad') imap.y('q_rps')];
linear.eulers.ss_shortperiod = getSubsystem(linear.eulers.ss_full, ...
    shortperiodstates, shortperiodinputs, shortperiodoutputs);

ix = [imap.x('alpha_rad') imap.x('theta_rad') imap.x('q_rps')];
iu = [imap.u('de_rad')];
iy = [imap.y('alpha_rad') imap.y('theta_rad') imap.y('q_rps')];
linear.eulers.ss_pitch_withoutwind = getSubsystem(linear.eulers.ss_full, ...
    ix, iu, iy);

rollstates = [imap.x('phi_rad') imap.x('p_rps')];
rollinputs = [imap.u('da_rad')];
rolloutputs = [imap.y('phi_rad') imap.y('p_rps')];
linear.eulers.ss_roll = getSubsystem(linear.eulers.ss_full, ...
    rollstates, rollinputs, rolloutputs);

% Longitudinal outer loop subsystem (vz_NED instead of alpha)
% Indices of output ports that are selected as new states:
ix = [1 16 3:5 7:12 15]; % vz
[linear.eulers_position.ss_full, linear.eulers_position.trim, linear.eulers_position.imap] = uavsimblockset_linearize(trimmod, ...
    xTrim, uTrim, yTrim, ...
    ix);
% Map that links state/output/input names to indices:
imap = linear.eulers_position.imap;
% Extract submodels:
longistates = [imap.x('Va_mps') imap.x('vz_NED_mps') imap.x('theta_rad') imap.x('q_rps') imap.x('x_NED_m') imap.x('z_NED_m')];
longiinputs = [imap.u('de_rad') imap.u('den_0to1') imap.u('vw_x_mps') imap.u('vw_z_mps') imap.u('omegaw_y_rps')];
longioutputs = [imap.y('Va_mps') imap.y('vz_NED_mps') imap.y('theta_rad') imap.y('q_rps') imap.y('x_NED_m') imap.y('z_NED_m')];
linear.eulers_position.ss_longi_position = getSubsystem(linear.eulers_position.ss_full, ...
    longistates, longiinputs, longioutputs);
linear.eulers_position.ss_longi_withoutwind = getSubsystem(linear.eulers_position.ss_full, ...
    longistates, [imap.u('de_rad') imap.u('den_0to1')], longioutputs);

% Longitudinal outer loop model that has theta as input:
x = [imap.x('Va_mps') imap.x('vz_NED_mps') imap.x('theta_rad') imap.x('x_NED_m') imap.x('z_NED_m')];
u = [imap.u('den_0to1')];
y = [imap.y('Va_mps') imap.y('vz_NED_mps') imap.y('x_NED_m') imap.y('z_NED_m')];
sys = getSubsystem(linear.eulers_position.ss_full, x, u, y);
linear.eulers_position.ss_longi_withoutwind_thetainput = x2u(sys, [3]);

% Lateral outer loop model that has phi as input:
x = [imap.x('y_NED_m') imap.x('vy_NED_mps') imap.x('phi_rad')];
u = [];
y = [imap.y('y_NED_m') imap.y('vy_NED_mps')];
sys = getSubsystem(linear.eulers_position.ss_full, x, u, y);
linear.eulers_position.ss_lat_withoutwind_phiinput = x2u(sys, [3]);

try
    ix = [imap.x('Va_mps') imap.x('vz_NED_mps') imap.x('theta_rad') imap.x('q_rps') ];
    iu = [imap.u('de_rad') imap.u('den_0to1') imap.u('vw_x_mps') imap.u('vw_z_mps') imap.u('omegaw_y_rps')];
    iy = [imap.y('nx') imap.y('nz') imap.y('Va_mps') imap.y('vz_NED_mps') imap.y('theta_rad') imap.y('q_rps')];
    linear.eulers_position.ss_nx = getSubsystem(linear.eulers_position.ss_full, ...
        ix, iu, iy);
catch e
    disp([mfilename '>> Exception: ' e.message]);
end

try
    imap = linear.eulers.imap;
    ix = [imap.x('alpha_rad') imap.x('q_rps')];
    iu = [imap.u('de_rad') imap.u('vw_z_mps') imap.u('omegaw_y_rps')];
    iy = [imap.y('nz') imap.y('alpha_rad') imap.y('q_rps')];
    linear.eulers.ss_nz = getSubsystem(linear.eulers.ss_full, ...
        ix, iu, iy);
catch e
    disp([mfilename '>> Exception: ' e.message]);
end

try
    % Generate models with vx_NED as state:
ix = [14 16 3:5 7:12 15];
[linear.eulers_NED.ss_full, linear.eulers_NED.trim, linear.eulers_NED.imap] = uavsimblockset_linearize(trimmod, ...
    xTrim, uTrim, yTrim, ...
    ix);
% Map that links state/output/input names to indices:

    imap = linear.eulers_NED.imap;
    ix = [imap.x('vx_NED_mps')];
    iu = [imap.u('den_0to1') imap.u('vw_x_mps')];
    iy = [imap.y('nx') imap.x('vx_NED_mps')];
    linear.eulers_NED.ss_nx = getSubsystem(linear.eulers_NED.ss_full, ...
        ix, iu, iy);
catch e
    disp([mfilename '>> Exception: ' e.message]);
end

% Remove small elements that are merely numerical artifacts:
linear.eulers = cleanup(linear.eulers);
linear.eulers_position = cleanup(linear.eulers_position);

    function cleaned = cleanup(var)
        cleaned = var;
        flds = fields(cleaned);
        for i=1:length(flds)
            if ~isempty(findstr('ss', flds{i}))
                system = cleaned.(flds{i});
                system.a = fflib_removeSmallElements(system.a, 1e-6);
                system.b = fflib_removeSmallElements(system.b, 1e-6);
                system.c = fflib_removeSmallElements(system.c, 1e-6);
                system.d = fflib_removeSmallElements(system.d, 1e-6);
                cleaned.(flds{i}) = system;
            end
        end
    end

end
