% Returns the transfer function of a second order low pass filter.
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function tfct = fflib_build2ndOrderLowpass(omega_n, damping, dcgain)
% num = dcgain * omega_n^2;
% den = [1 2*damping*omega_n omega_n^2];
num = dcgain * omega_n^2;
den = [1 2*damping*omega_n omega_n^2];

tfct = tf(num, den);
end