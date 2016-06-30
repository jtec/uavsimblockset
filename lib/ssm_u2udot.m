% Adds integrators at selected inputs of a given state space model. This
% makes the input derivatives the new inputs.
function ssmudot = ssm_u2udot(ssm, iu)
% Identify relevant input names:
names = ssm.InputName(iu);
% Build integrator system.
% The inputs of the integrator system are all inputs of the original
% system, the states are the inputs to be integrated. These pass by C to
% the outputs, the other ones by D;
a = zeros(length(iu));

b = zeros(length(iu), length(ssm.u));
for k=1:length(iu)
    b(k, iu(k)) = 1;
end

c = zeros(length(ssm.u), length(iu));
for k=1:length(iu)
    c(iu(k), k) = 1;
end

ddiag = ones(length(ssm.u), 1);
ddiag(iu) = 0;
d = diag(ddiag);

intsys = ss(a, b, c, d);
for k=1:length(intsys.StateName)
    intsys.StateName{k} = [names{k} '_int'];
end
intsys.InputName = ssm.InputName;
for k=1:length(iu)
    intsys.InputName{iu(k)} = [intsys.InputName{iu(k)} '_dot'];
end
intsys.OutputName = ssm.InputName;
ssmudot = ssm * intsys;
end