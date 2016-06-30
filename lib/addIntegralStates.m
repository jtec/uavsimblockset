% Adds states to a state space system that are the integral of some other
% states.
function augsys = addIntegralStates(sys, ix)
% Augment A:
a = [[sys.a zeros(length(sys.a), length(ix))];
    zeros(length(ix), length(sys.a) + length(ix))];
augstatenames = {};
for i=1:length(ix)
    a(end-length(ix)+i, ix(i)) = 1;
    augstatenames{end+1} = [sys.StateName{ix(i)} '_Int'];
end
% Augment B:
b = [sys.b; zeros(length(ix), size(sys.b, 2))];
% Augment C, add integral states as outputs:
c = [sys.c zeros(size(sys.c, 1), length(ix))];
c = [c; [zeros(length(ix), size(sys.c, 2)) eye(length(ix))]];
% Augment D:
d = [sys.d; zeros(length(ix), size(sys.d, 2))];
% Build augmented system:
augsys = ss(a, b, c, d);
augsys.StateName = [sys.StateName; augstatenames'];
augsys.OutputName = [sys.OutputName; augstatenames'];
augsys.InputName = sys.InputName;

end