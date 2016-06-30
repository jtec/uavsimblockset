% Returns the transfer function of a second order low pass filter.
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function tfc = fflib_build2ndOrderHighpass(omega_n, damping, dcgain)
num = [dcgain 0 0];
den = [1 2*damping*omega_n omega_n^2];
tfc = tf(num, den);
end