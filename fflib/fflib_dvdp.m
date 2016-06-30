% Returns the gradient [m/s / Pa] of the indicatied airspeed towards the
% static pressure measurement, assuming that the air density is computed using the ideal
% gas law, and the IAS is computed according to IAS = sqrt(2*qbar*R*T/p).
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function dvdp= fflib_dvdp(p, qBar, T)
R_dryair = 287.058;

d = 1e-6;
rho1 = p/(R_dryair*T);
rho2 = p*(1+d)/(R_dryair*T);

v1 = fflib_qbar2mps(qBar, rho1);
v2 = fflib_qbar2mps(qBar, rho2);
dvdp = (v2 - v1) / (p*d);

end