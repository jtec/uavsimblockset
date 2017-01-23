function ssmout = ssm_y2x(ssmin, ny2x)
% Calculate transformation matrix and convert to the new system:
C_x_bar = ssmin.c(1:ny2x, 1:ny2x);
ssm_newStates = ss2ss(ssmin, C_x_bar);
ssm_newStates.StateName = ssmin.OutputName(1:ny2x);
ssmout = ssm_newStates;
end