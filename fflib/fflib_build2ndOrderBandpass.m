% Returns the transfer function of a second order low pass filter.
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function [num, den] = fflib_build2ndOrderBandpass(omega_n, damping, dcgain)
num = [2*damping*omega_n 0];
den = [1 2*damping*omega_n omega_n^2];
%bode(tf(num, den));
end