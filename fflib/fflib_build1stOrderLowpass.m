% Returns the transfer function of a first order low pass filter.
%
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function ssm = fflib_build1stOrderLowpass(T, dcgain)
ssm = ss(-1/T, 1/T, dcgain, 0);
end