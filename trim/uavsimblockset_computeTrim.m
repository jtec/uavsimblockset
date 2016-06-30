function [x_trim, u_trim, y_trim, residual] = uavsimblockset_computeTrim(filename, chi0, gamma0, Va)
% This function returns the trim states and inputs to a desired trim point.
% It is fed with a trim point (chi, gamma, Va) and sets all the remaining
% quantities such as xDot in an appropriate fashion before calling the
% matlab 'trim' function to compute the states and inputs. Consider it as a
% covenience wrapper of the matlab 'trim' function.

% Inputs:
% FIXME: do it like a man - set units to all quantites!
%  - 'filename'  which is the name of the simulink model that is to be
%     trimmed
%
%  - Trim point defined by the 3 quantities:
%    Va    = airspeed
%    gamma = flight path angle
%    R     = radius of the orbit(in case of non symmetrical trim point)
%
%
% Author: Martin Stolle
% ONERA Toulouse
% email address: martin.stolle@mail.com
% January 2014; Last revision: 12-January-2014

%------------- BEGIN CODE --------------

% Set state derivative:
%  Compute pDot in NED:
% Get unit vector in the NED xy plane pointing in the direction of the
% flight path azimuth:
unit = [1 0 0];
% Rotate by chi around NED z axis:
qChi = RotationMath.angleaxis2quat([0; 0; 1], chi0)';
unit = quatrotate(qChi, unit);
newRotationAxis= cross(unit, [0 0 1])';
qGamma = RotationMath.angleaxis2quat(newRotationAxis, gamma0)';

unit = quatrotate(qGamma, unit);
uvw_NED_mps = unit * Va;

xDot0  = [zeros(3,1);    % uvwDot_b_mps2
    zeros(3,1);    % eulerDot_rps
    uvw_NED_mps';  % xyzDot_NED_mps
    zeros(3,1)];   % pqrDot_rps2

% Selects the values in the state derivatives that must be satisfied.
ixDot0 = [1:12'];

% Initial guesses for Trim - state, input and output:
x0  = [[Va 0 0]';       % uvw_mps - close guess since alpha and beta are usually small.
    zeros(3,1);      % euler_rad
    zeros(3,1);      % xyz_NED_m
    zeros(3,1)];     % pqr_rps

ix0 = [10:12];

%     [de da dr den df 3xV_w 3xOmega_w]
u0  = [0; 0; 0; 1; 0; zeros(6,1)];
% No flaps, no turbulence for trim:
iu0 = [5, 6:11]';
%     [V   alpha beta  eulers_rad     p_NED_m  omega nz vNED gamma chi]
y0  = [Va; 0;    0;    zeros(3,1);    zeros(3,1);  zeros(3,1); 0; zeros(3,1); 0; 0];
iy0 = [1,3];

% Call the matlab 'trim' function:
[x_trim,u_trim,y_trim,dx_trim, options] = trim(filename,x0,u0,y0,ix0,iu0,iy0,xDot0,ixDot0);

residual = options(8);
% Check to make sure that the linearization worked (should be small)
display([mfilename '>> trimResidual = ' num2str(residual)]);

end