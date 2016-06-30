% Sample incremental lift over y/z separation:
b = uavsim.cularis.b;
L = 9.81*uavsim.cularis.mass;
N = [1, 40, 40];
dp = buildSet([-2*b, 0, 0], [-2*b, -b/2, -b/2], [-2*b, b/2, b/2], N);
deltaL = [];
for k=1:length(dp)
    deltaL(end+1) = HSVM(dp(k, :)', uavsim.cularis.mass, 15, 1.225, uavsim.cularis.b, uavsim.cularis.CL_alpha, uavsim.cularis.c);
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
        Z(kx, ky) = deltaL(p)/L;
        p = p+1;
    end
end

%%
surf(X, Y, Z);
c = colorbar;
c.Label.String = 'normalized incremental lift';
c.Location = 'northoutside';
%plot3(dy, dz, deltaL/L, '*');
xlabel('$\frac{\Delta y''}{b}$');
ylabel('$\frac{\Delta z''}{b}$');
zlabel('$\frac{\Delta L''}{W}$');
grid on;

%%
exportfigure2latex(gcf, ['incrementallift']);