% Returns the closest integer multiple of the second argument that is equal
% or higher than the first argument.
function mpl = findclosemultiple(approx, base)
mpl = base*(ceil(approx/base));
end