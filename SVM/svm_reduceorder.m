function ssm_red = svm_reduceorder( T, n, dcggain)
T = tf(T);
% Set DC gain of original system:
ggain = dcgain(T);
T = dcggain * ggain^-1 * T;

figure(1);
hsvd(T);
ssm_red = hankelmr(T, n);
%sys = ltiblock.ss('red', n, length(T.OutputName), length(T.InputName));
%[CL,gamma,info] = hinfstruct(sys-T);

% Set DC gain of reduced system:
%ssm_red = makeCUnity(ssm_red);
%ggain = ssm_red.D - ssm_red.c*ssm_red.a^-1*ssm_red.b;
%ssm_red.b = dcggain * ggain^-1 * ssm_red.b;
ssm_red = (1/dcgain(ssm_red))*ssm_red;

figure(2);
subplot(2,1,1);
opt = sigmaoptions;
opt.MagUnits = 'abs';
sigmaplot(T, opt);
hold on;
sigmaplot(ssm_red, opt);
sigmaplot(ssm_red - T, opt);
grid on;
legend({'full order', ['order ' num2str(n)], 'difference'});
subplot(2,1,2);
step(T);
hold on;
grid on;
%[nx, ny, nu] = sssizes(T);
step(ssm_red);
legend({'full order', 'reduced order'});

ssm_red = tf(ssm_red);
end