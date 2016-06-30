% Sets matrix elements whose absolute value is smaller than a certain
% threshold to zero.
% \author Jan Bolting, ONERA/ISAE, 2014, j.bolting@isae.fr
%
function aOut = fflib_removeSmallElements(a, thr)
a(abs(a)<thr) = 0;
aOut = a;
end
