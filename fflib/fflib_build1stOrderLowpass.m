% Returns the transfer function of a first order low pass filter.
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function tfct = fflib_build1stOrderLowpass(T, dcgain)
num = dcgain;
den = [T 1];
tfct = tf(num, den);
end