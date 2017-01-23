% Returns 1 if a is an integer multiple of b, 0 otherwise;
function yesorno = ismultipleof(a, b)
yesorno = (rem(a,b)==0);
end