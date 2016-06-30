% Returns the gradient [m/s / °K] of the indicatied airspeed towards the air
% temperature measurement, assuming that the air density is computed using the ideal
% gas law, and the IAS is computed according to IAS = sqrt(2*qbar*R*T/p).
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function dvdT = fflib_dvdT(p, qBar, T)
R_dryair = 287.058;
% dvdT = sqrt(R_dryair*qBar/2)* (T*p)^(-0.5);
R_dryair = 287.058;

d = 1e-6;
rho1 = p/(R_dryair*T);
rho2 = p/(R_dryair*T*(1+d));

v1 = fflib_qbar2mps(qBar, rho1);
v2 = fflib_qbar2mps(qBar, rho2);
dvdT = (v2 - v1) / (T*d);

end