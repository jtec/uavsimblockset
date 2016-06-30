% Changes the output order of a given state space model.
function ssmp = yperm(ssmi, idx)
ssmp = ssmi;
c = zeros(size(ssmi.c));
d = zeros(size(ssmi.d));
names = ssmi.OutputName;

for k=1:length(names)
    names(k) = ssmi.OutputName(idx(k));
    c(k,:) = ssmi.c(idx(k), :);
    d(k,:) = ssmi.d(idx(k), :);
end
ssmp.c = c;
ssmp.d = d;
ssmp.OutputName = names;
end