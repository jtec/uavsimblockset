% Sample incremental lift over y/z separation:
b = uavsim.cularis.b;
L = 9.81*uavsim.cularis.mass;
N = [1, 40, 40];
dp = buildSet([-2*b, 0, 0], [-2*b, -b/2, -b/2], [-2*b, b/2, b/2], N);
deltaL = [];
deltaP = [];

%Problem data:
situation.Vinf = 15;
situation.rho = 1.225;
% Predecessor data:
pred.b = uavsim.cularis.b;
pred.c = uavsim.cularis.c;
pred.S = uavsim.cularis.S;
pred.m = uavsim.cularis.mass;
% Assuming cruise flight:
pred.CL = (2*pred.m*9.81)/(situation.rho*situation.Vinf^2*pred.S);
% Follower data:
foll.b = uavsim.cularis.b;
foll.CL_alpha = uavsim.cularis.CL_alpha;
foll.c = uavsim.cularis.c;
foll.m = uavsim.cularis.mass;
foll.Flift = uavsim.cularis.mass;
situation.foll = foll;
situation.pred = pred;
Fdrag0 = 2.5;
for k=1:length(dp)
    situation.dp_w2w = dp(k, :)';
    %situation.dp_w2w = [-2*b; 0.001; 0.001];
    [deltaL(end+1) deltaP(end+1)] = HSVM(situation);
end
deltaL = deltaL';

% Generate matrices required by matlab for surface plots:
dx = dp(:, 1);
dy = dp(:, 2);
dz = dp(:, 3);
X = [];
Y = [];
Z = [];
p = 1;
for kx = 1:N(2)
    for ky = 1:N(3)
        X(kx, ky) = dy(p)/b;
        Y(kx, ky) = dz(p)/b;
        %Z(kx, ky) = deltaL(p)/L;
        Z(kx, ky) = -deltaP(p)/(Fdrag0*situation.Vinf);
        p = p+1;
    end
end

%
surf(X, Y, Z);
c = colorbar;
c.Label.String = 'normalized incremental propulsion power';
c.Location = 'northoutside';
%plot3(dy, dz, deltaL/L, '*');
xlabel('$\frac{\Delta y''}{b}$');
ylabel('$\frac{\Delta z''}{b}$');
zlabel('$\frac{\Delta P''}{W}$');
grid on;

%%
exportfigure2latex(gcf, ['incrementallift']);