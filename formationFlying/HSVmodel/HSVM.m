% Horseshoe vortex model for incremental lift induced by
% trailing aircraft. Takes only the closest vortex into account.
function [deltaL ] = HSVM(dp, Lpred, Vinf, rho, b, Cl_alpha, c)
% Vortex strength:
Gamma = 9.81*Lpred/(rho*Vinf*b);

% Grid over span: 
s = linspace(0,b,50);

deltaL = 0;
for k=1:length(s)
    % Point where velocity is computed:
    P = dp + s(k)* [0 1 0]';
    % Radius to this point:
    r = norm(P(2:3));
    % Tangential vector at this point in the vortex:
    theta = cross([-1 0 0]', [0; P(2:3)]);
    % Normalize:
    theta = theta/norm(theta);
    % Induced velocity at this point:
    % using core model:
    tau = norm(dp(1))/Vinf;
    rC = 2.24*sqrt(0.06*Gamma * tau);
    rR = r;
    q = (Gamma/(2*pi)) * rR/(rR^2 + rC^2) * theta;
    % Component normal to the followers wing:
    w = q(3);
    deltaalpha = atan(w/Vinf);
    deltaS = c*b/length(s);
    deltaL = deltaL - deltaS * Cl_alpha * deltaalpha * ((rho/2)*Vinf^2);
end

end