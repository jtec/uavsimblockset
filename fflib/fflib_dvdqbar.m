% Returns the gradient [m/s / Pa] of the indicated airspeed towards the measured
% dynamic pressure measurement, assuming that the air density is computed using the 
% ideal gas law, and the IAS is computed according to 
% IAS = sqrt(2*qbar*R*T/p).
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function dvdqbar = fflib_dvdqbar(p, qBar, T)
R_dryair = 287.058;
%dvdqbar = sqrt(R_dryair*T/2) * (p*qBar)^(-0.5);

d = 1e-6;
rho = p/(R_dryair*T);
qBar1 = qBar;
qBar2 = qBar*(1+d);

v1 = fflib_qbar2mps(qBar1, rho);
v2 = fflib_qbar2mps(qBar2, rho);
dvdqbar = (v2 - v1) / (qBar*d);

end
