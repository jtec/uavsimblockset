% Adds outputs to a given state space system
function ssmout = ssm_addy(ssmin, ix, crows, names)
a = ssmin.a;
b = ssmin.b;
c = [ssmin.c; crows];
d = [ssmin.d; zeros(length(ix), length(ssmin.InputName))];
ssmout = ss(a, b, c, d);
ssmout.StateName = ssmin.StateName;
ssmout.InputName = ssmin.InputName;
ssmout.OutputName = [ssmin.OutputName; names];
end