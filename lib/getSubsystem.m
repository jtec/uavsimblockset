% Returns a subsystem of a given state space system as a function of
% the given state, input and output vector indices.
function subsystem = getSubsystem(sssystem, states, inputs, outputs)
nStates = length(sssystem.a);
statesToBeEliminated = (1:nStates)';
statesToBeEliminated(states) = [];
subs = sssystem(outputs, inputs);
subsystem = modred(subs, statesToBeEliminated, 'Truncate');
end

