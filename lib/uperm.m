% Changes the input order of a given state space model.
function ssmp = uperm(ssmi, idx)
ssmp = ssmi;
b = zeros(size(ssmi.b));
d = zeros(size(ssmi.d));
names = ssmi.InputName;

for k=1:length(names)
    names(k) = ssmi.InputName(idx(k));
    b(:,k) = ssmi.b(:, idx(k));
    d(:,k) = ssmi.d(:, idx(k));
end
ssmp.b = b;
ssmp.d = d;
ssmp.InputName = names;
end