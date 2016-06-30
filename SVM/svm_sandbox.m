%% Compute error measures and plot:
suffixes = {'PDSMC', 'DSMC', 'TSCSMC'};
bobob = {'-', '--', '*', '+', '.', 'x'};
h = figure(44);
hold off;
cnt = 1;
resultstoplot = {};
lgd = {};
for kplot=1:length(suffixes)
    cmd = ['resultstoplot{kplot} = load(''' suffixes{kplot} '-100Hz-TIMESCALESEPARATION-turbulence=1-Tau-vs-error'');']
    eval(cmd);
    pltr = resultstoplot{kplot};
    h = figure(44);
    err = [];
    errWOmean = [];
    rmserror = [];
    
    for n=1:length(pltr.results_taugrid)
        dp = pltr.results_taugrid{n}.follower_dp.Data(:,3);
        ni = 1; %ceil(length(dp)/4);
        err(end+1) = max(abs(dp))
        cleanedup = dp(ni:end);
        cleanedup = cleanedup - mean(cleanedup);
        errWOmean(end+1) = max(abs(cleanedup));
        rmserror(end+1) = rms(cleanedup);
    end
    
    % Plot:
    hold on;
    %subplot(2,1,1);
    plot(pltr.tau(1:end), err/uavsim.cularis.b, bobob{cnt});
    cnt = cnt + 1;
    plot(pltr.tau(1:end), rmserror/uavsim.cularis.b, bobob{cnt});
    cnt = cnt + 1;
    grid on;
    xlabel('T  [s]');
    ylabel('$\frac{\Delta z}{b}$ [-]');
    %legend({'max error'});
    %subplot(2,1,2)
    %plot(tau, errWOmean, '*');
    %grid on;
    %
    lgd = [lgd {['max '  suffixes{kplot}], ['rms '  suffixes{kplot}] }]
end
legend(lgd, 'Location', 'northwest');
set(h, 'Position', [100, 100, 320, 410]);
return;

%% Export to pdf:
exportfigure2latex(gcf, ['error_z_vs_samplingtime']);

%% Plot load factor perturbations in z
h = figure(987);
plot(follower_nw.n.Time, follower_nw.n.Data(:, 3), 'k');
grid on;
xlabel('time [s]');
ylabel('$n_{w,3} [-]$');
set(h, 'Position', [100, 100, 320, 150]);
%%
exportfigure2latex(h, ['nwz-timeseries']);

%% Sample max tracking error over sampling time grid:
results_taugrid = {};
% Simulate open-loop to generate leader trajectory:
svm.dp0 = [0 0  (uavsim.cularis.b)]';
svm.maneuvergain = 1;
svm.p0 = [0 0 -50 + (uavsim.cularis.b)];
postracking.T = 1e-3;
uavsim.svm.tsample_model = postracking.T;
sim('svm_test');
predec_x = follower_x;
taugrid = 1e-3:1e-3:1e-2;
tau = linspace(1e-3,1e-2, 50);

svm.turbulenceOn = 1;
svm.turbulenceseed = '[1 2 3 4]';
controller = 'TSCSMC';
innerloops = innerloopmodes(num2str(svm_par_innerloopmode));
turbulent = num2str(svm.turbulenceOn);
conditions = [controller '-' num2str(1/postracking.T) 'Hz-' innerloops '-turbulence=' turbulent '-' notes];

for n=1:length(tau)
    T = ceil(tau(n)/1e-4)*1e-4
    % Make dryden sampling time integer multiple of model sampling time:
    uavsim.svm.tsample_dryden = ceil(0.1/T)*T;
    uavsim.svm.tsample_uav3d = ceil(0.03/T)*T;
    
    postracking.T = T;
    uavsim.svm.tsample_model = T;
    tic;
    disp([mfilename '>> Running benchmark maneuver for T=' num2str(postracking.T) ' s']);
    sim('svm_sim');
    results_taugrid{end+1}.follower_x = follower_x;
    results_taugrid{end}.follower_u = follower_u;
    results_taugrid{end}.follower_dp = follower_dp;
    toc
    n
end

save([conditions '-Tau-vs-error']);
return;



%% Debug prediction stage of DSMC:
p = follower_x.p.Data;
v = follower_x.v.Data;
a = follower_accNED.Data;
usat = follower_usat.n.Data;
u = follower_u.n.Data;
T = follower_x.p.Time(end) - follower_x.p.Time(end-1);
clc;
format long;
for k=200:length(p);
    p_kminusone = p(k-1, :)';
    v_kminusone = v(k-1, :)';
    u_kminusone = usat(k-1, :)';
    a_kminusone = a(k-1, :)';
    a_kminusone_pred = 9.81*(u_kminusone + [0 0 1]');
    
    p_k_pred = p_kminusone + T*v_kminusone;
    v_k_pred = v_kminusone + T*a_kminusone_pred;
    
    %v_k_pred = v_kminusone + T*a_kminusone;
    
    p_k = p(k, :)';
    v_k = v(k, :)';
    x = [p_k; v_k];
    x_pred = [p_k_pred; v_k_pred];
    
    Ax = [eye(3) T*eye(3);
        zeros(3) eye(3)];
    Bx = T*[zeros(3); 9.81*eye(3)];
    ng = [0 0 1]';
    x_kminusone = [p_kminusone; v_kminusone];
    %x_pred = Ax*x_kminusone + Bx*(u_kminusone + ng);
    dx_pred = x_pred - x_kminusone;
    dx = x - x_kminusone;
    predictionerror_x = (x_pred - x)./x;
    predictionerror_dx = (dx_pred - dx)./dx;
    predictionerror_a = (a_kminusone - a_kminusone_pred)./a_kminusone
    
    if max(abs(predictionerror_a)) > 10*eps
        predictionerror_a;
    end
end
return;
%% Determine max. perturbations due to turbulence:
%sim('svm_turbulencebounds');
nx = simout.n.Data(:, 1);
nz = simout.n.Data(:, 3);
svm.turbulence.nz_max = max(abs(nz));
svm.turbulence.nx_max = max(abs(nx));



%% Test with theta TF:
test = fflib_build2ndOrderLowpass(10, 0.8, 1);
hold off;
step(test);
hold on;
step(uavsim.cularis.linearModels.eulers.ss_theta_cl_reduced);
uavsim.cularis.linearModels.eulers.ss_theta_cl_reduced = test;
uavsim.cularis.linearModels.eulers.ss_nz_cl_sorder = uavsim.cularis.linearModels.eulers.ss_theta_cl_reduced;

%% Plot magnitude over frequency of inner loop output errors in z:
figure(998);
w = {1e-1, 1e3};
opt = sigmaoptions;
opt.MagUnits = 'abs';
opt.FreqUnits = 'Hz';
subplot(2,1,1);
hold off;
sigmaplot(uavsim.cularis.linearModels.eulers.ss_nz_cl_forder - 1, w, opt);
hold on;
sigmaplot(uavsim.cularis.linearModels.eulers.ss_nz_cl_sorder - uavsim.cularis.linearModels.eulers.ss_nz_cl_forder, w, opt);
sigmaplot(T_nzc2nz - uavsim.cularis.linearModels.eulers.ss_nz_cl_forder, w, opt);
sigmaplot(s*uavsim.cularis.linearModels.eulers.ss_nz_cl_forder - s, w, opt);
grid on;
title('')
legend('1st order - 0 order', '2nd order - 1st order', 'full order - 1st order', '1st order dot- 0 order dot');

subplot(2,1,2);
hold off;
sigmaplot(tf(1), w, opt);
hold on;
sigmaplot(uavsim.cularis.linearModels.eulers.ss_nz_cl_forder, w, opt);
sigmaplot(uavsim.cularis.linearModels.eulers.ss_nz_cl_sorder, w, opt);
sigmaplot(T_nzc2nz, w, opt);
%sigmaplot(uavsim.cularis.linearModels.eulers.ss_nz_cl_forder*s, w, opt);
%sigmaplot(s, w, opt);
grid on;
title('')
legend('0 order', '1st order', '2nd order', 'full order');

%%
exportfigure2latex(gcf, ['innerloopdegree_magnitude_nz']);

%% Evaluate sliding surface:
lambda = [-6; -4; 1];
a = [0 1;
    lambda(1)/lambda(3) lambda(2)/lambda(3)];
ssf = ss(a, [], eye(2), []);
initial(ssf,[1 0],10);
eig(ssf)

%% Plot separation errors:
% Load data?
loaded = load('TSCSMC-1000Hz-TIMESCALESEPARATION-turbulence=1-lightweight=0');

conditions = loaded.conditions;
results = loaded.results;
uavsim = loaded.uavsim;
linestyles = {'-o',...
              '--',...
              ':',...
              '-.',...
              '-*',...
              '-*'}
try
    close(1);
catch
end
figure(1);

lgd = {};
nsplots = 3;
N = 6;
colors = linspecer;
lw = 1;
% Re-sample for more lightweight figures:
ts = results{1}.follower_x.p.Time(1):0.2:results{1}.follower_x.p.Time(end);
results_downsampled = results(1:end);
for k=1:length(results)
    results_downsampled{k}.follower_x.p = resample(results_downsampled{k}.follower_x.p, ts);
    results_downsampled{k}.follower_x.v = resample(results_downsampled{k}.follower_x.v, ts);
    results_downsampled{k}.follower_u.n = resample(results_downsampled{k}.follower_u.n, ts);
    results_downsampled{k}.follower_usat.n = resample(results_downsampled{k}.follower_usat.n, ts);
    if k >=2
        results_downsampled{k}.follower_dxc = resample(results_downsampled{k}.follower_dxc, ts);
    end
end
cnt = 1;
for k=3:N+1
    results_downsampled{k}.follower_x
    figure(1);
    if nsplots >= 1
        subplot(nsplots,1,1);
        t = results_downsampled{k}.follower_x.p.Time;
        dp = results_downsampled{k}.follower_x.p.Data(:,3) ...
            - results_downsampled{k-1}.follower_x.p.Data(:,3) ...
            -  results_downsampled{k}.follower_dxc.Data(:, 3);
        h1 = plot(t, dp/uavsim.cularis.b, linestyles{cnt}, 'Color', colors(cnt, :), 'LineWidth', lw);
        hold on;
        grid on;
        %xlabel('');
        ylabel('$\frac{\Delta z}{b}$ [-]');
        lgd{end+1} = ['#' num2str(k-2)];
        if k == ceil(length(results_downsampled)/2)
            lgd{end} =  sprintf('%s\n', lgd{end});
        end
    end
    if nsplots >= 2
        subplot(nsplots,1,2);
        p = results_downsampled{k}.follower_x.p.Data(:,3);
        h2 = plot(t, p, linestyles{cnt}, 'Color', colors(cnt, :), 'LineWidth', lw);
        hold on;
        grid on;
        %xlabel('time [s]');
        ylabel('z [m]');
    end
    if nsplots >= 3
        subplot(nsplots,1,3);
        h3 = plot(results_downsampled{k}.follower_usat.n.Time, results_downsampled{k}.follower_usat.n.Data(:,3) / uavsim.svm.usat.n_c.upper(3), linestyles{cnt}, 'Color', colors(cnt, :), 'LineWidth', lw);
        hold on;
        grid on;
        xlabel('time [s]');
        ylabel('$\frac{n_{z,c}}{n_{z,max}}$ [-]');
    end
    cnt = cnt + 1;
end
h = figure(1);
set(h, 'Position', [100, 100, 560, 550]);
subplot(nsplots,1,1);
legend(lgd, 'Orientation', 'horizontal', 'Location', 'north');
% Align subplots
drawnow;
return;

%% Export figure to pdf:
exportfigure2latex(figure(1), [conditions]);
return;

%% Run sequential simulation:
N = 6;
results = {};
% Simulate open-loop to get first leader trajectory:
svm.p0 = [0 0 -50];
svm.turbulenceOn = 1;
svm.wakeON = 0;
svm.maneuvergain = 1;
svm.tsim = 20;
postracking.T = 1e-2;
uavsim.svm.tsample_model = postracking.T;
controller = 'PDSMC';
innerloops = innerloopmodes(num2str(svm_par_innerloopmode));
turbulent = num2str(svm.turbulenceOn);
svm.turbulenceseed = 'randi(2^6, [1 4])';
notes = ' '; % 'turbulenceonlyfirstUAS'];
conditions = [controller '-' num2str(1/postracking.T) 'Hz-' innerloops '-turbulence=' turbulent '-' notes];

sim('svm_test');
results{end+1}.follower_x = follower_x;
results{end}.follower_u = follower_u;
results{end}.follower_usat = follower_usat;
predec_x = follower_x;
golightweight = false;
results_lightweight = {};
for n=1:N
    tic;
    % svm.p0 = [0 n*uavsim.cularis.b -50 + (n*uavsim.cularis.b)];
    svm.p0 = [0 0 -50 + (n*uavsim.cularis.b)];
    %svm.maneuvergain = 0;
    sim('svm_sim');
    % Switch turbulence off after the first run to induce errors only in
    % the first vehicle pair and then observe error propagation 
    %svm.turbulenceOn = 0;
    % Save all or just the essentials:
    if golightweight
        results_lightweight{end+1}.follower_u = follower_u;
        results_lightweight{end}.follower_dp = follower_dp;
    else
        results{end+1}.follower_x = follower_x;
        results{end}.follower_u = follower_u;
        results{end}.follower_usat = follower_usat;
        results{end}.sigma_z = sigma_z;
        results{end}.follower_dxc = follower_dxc;
    end
    predec_x = follower_x;
    toc
    n
end

% Just save the whole workspace, simulation results make up the most part
% of it anyway.
save(['storage/' conditions '-lightweight=' num2str(golightweight)]);

%% Analyse error amplification:
%load('TSCSMC-100Hz-TIMESCALESEPARATION-turbulence=1#turbulenceonlyfirstUAS-lightweight=1');
N = length(results_lightweight)
N = 30
emax = zeros(N,1);
erms = emax;
for k=1:N
   dz = results_lightweight{k}.follower_dp.Data(:, 3);
   t = results_lightweight{k}.follower_dp.Time;
   u = results_lightweight{k}.follower_u.n.Data(:, 3);
   erms(k) = rms(dz);
   emax(k) = max(abs(dz));
end
figure(3312);
hold off;
plot(1:N, erms, '.');
hold on;
plot(1:N, emax, '*');
grid on;
xlabel('vehicle index');
ylabel('$\Delta z [m]$');
legend({'rms', 'max'}, 'Location', 'northwest');
set(gcf, 'Position', [100, 100, 320, 220]);

%% Export figure to pdf:
exportfigure2latex(gcf, ['erroramplification-' conditions]);
return;

%% Build inner loop models derived from attitude loops:
% Get closed-loop attitude dynamics:
cularis_closedinnerloops_trimscript;
cl = linearize('trimModel_Cularis_turbulentAtmosphere_innerloopsclosed', op);
% Correct naming issue:
cl.u{4} = 'phi_c';
cl.y{4} = 'phi';
engine_cl = cl(1,1);
psi_cl = cl(2,2);
theta_cl = cl(3,3);
phi_cl = cl(4,4);
%step(phi_cl);

%% Generate reduced order models:
uavsim.cularis.linearModels.eulers.ss_theta_cl_reduced = svm_reduceorder(theta_cl, 2, 1);
uavsim.cularis.linearModels.eulers.ss_phi_cl_reduced = svm_reduceorder(phi_cl, 2, 1);
close('all');

%%
test = fflib_build2ndOrderLowpass(4, 0.8, 1);
hold off;
step(test);
hold on;
step(uavsim.cularis.linearModels.eulers.ss_theta_cl_reduced);
uavsim.cularis.linearModels.eulers.ss_theta_cl_reduced = test;
