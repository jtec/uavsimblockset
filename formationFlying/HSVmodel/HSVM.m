% Horseshoe vortex model for incremental lift induced by
% trailing aircraft. Takes only the closest vortex into account. BAsed on 
% "Modeling of Aerodynamic Coupling Between Aircraft in Close Proximity",
% Dogan et  al.
function [deltaL, deltaP] = HSVM(st)
% Vortex strength:
Gamma = 0.5*st.pred.CL*st.Vinf*st.pred.c;

% Grid over span: 
s = linspace(0,st.foll.b,50);

deltaL = 0;
deltaP = 0;
for k=1:length(s)
    % Point where velocity is computed:
    % The vortex is located at pi/4 of the predecessor's wingspan:
    
    P = [0; (1-pi/4)*st.pred.b; 0] + st.dp_w2w + s(k)* [0 1 0]';
    % Radius to this point:
    r = norm(P(2:3));
    % Tangential vector at this point in the vortex:
    theta = cross([-1 0 0]', [0; P(2:3)]);
    % Normalize:
    theta = theta/norm(theta);
    % Induced velocity at this point:
    % using core model:
    tau = norm(st.dp_w2w(1))/st.Vinf;
    rC = 2.24*sqrt(0.06*Gamma * tau);
    rR = r;
    q = (Gamma/(2*pi)) * rR/(rR^2 + rC^2) * theta;
    % Component normal to the follower's wing:
    w = q(3);
    deltaalpha = atan(w/st.Vinf);
    deltaS = st.foll.c*st.foll.b/length(s);
    deltaL = deltaL - deltaS * st.foll.CL_alpha * deltaalpha * ((st.rho/2)*st.Vinf^2);
    L = 9.81*st.foll.m;
    dL = L/length(s);
    % Compute drag reduction of this piece of the wing due to lift vector
    % being rotated by upwash:
    deltaD = dL*deltaalpha;
    deltaP = deltaP + (deltaD*st.Vinf);
end

end