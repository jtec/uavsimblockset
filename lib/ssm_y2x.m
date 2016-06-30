function ssmn = ssm_y2x(ssm, ny2x)
% Build new system with the desired states and the desired outputs:

% 1. Intermediate: Build new system with the desired states (keep same
%    outputs):
% Calculate transformation matrix and convert to the new system:
C_x_bar = ssm.c(1:ny2x, 1:ny2x);
ssm_newStates = ss2ss(ssm, C_x_bar);
ssm_newStates.StateName = ssm.OutputName(1:ny2x);

ssmn = ssm_newStates;
end