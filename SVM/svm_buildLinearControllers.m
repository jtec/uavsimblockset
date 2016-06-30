% Longitudinal position controller:
s = tf('s');
T_psderivative = s/(1e-6*s + 1);
M_psderivative = ss(blkdiag(T_psderivative, T_psderivative));
% Disturbance input transfers: wind speeds -> load factors
turbulence2loadfactors = blkdiag(T_w2nx, T_w2nz);
% 1 predecessor, relative position and relative speed available for
% feedback:
My_3D = ([eye(6) -eye(6)]);
My = ss(tf([eye(4) -eye(4)]));
% cost output selection matrix:
Mz = ss(tf([eye(2) zeros(2) -eye(2) zeros(2)]));
% Add integral of position error 
Mint = (tf([(1/s)*eye(2) zeros(2) -(1/s)*eye(2) zeros(2)]));
%My = [My; Mint];
%My = [My; zeros(size(Mint))];
Knx = 0;
Kny = 2;
Knu = 4;
%K = rss(Knx, Kny, Knu);
K = (ltiblock.ss('K',Knx,Kny,Knu));
% Feedforward:
Fnx = 0;

svm_buildTfs;
% 0.1 m tracking error, 0.1m coupling error, control effort
soft = [T_dist_slk; 10*blkdiag(W_uout, W_uout)*T_ceffort_dist_slk] * blkdiag(W_distin, W_distin, W_distin);
soft.InputName = {'vwx', 'vwz', 'omegawy'};
soft.OutputName = {'dpx', 'dpz', 'nxc', 'nzc'};
% Mesh stability:
hard = [T_mstab]* blkdiag(W_refin, W_refin);
hard.InputName = {'dpxin', 'dpzin'};
hard.OutputName = {'dpxout', 'dpzout'};
tfs = blkdiag(soft, hard);
%tfs = minreal(tfs);
%opt = hinfstructOptions('Display','final','RandomStart', 500, 'TargetGain', 1);
rng(213);
%[Ttuned,gamma,info] = hinfstruct(tfs , opt);
%K = ss(Ttuned.Blocks.K)

% Hard requirements: mesh stability
req_mstab = TuningGoal.Gain({'dpxin', 'dpzin'}, {'dpxout', 'dpzout'}, 1);  
% Soft requirements:
req_distrej = TuningGoal.Variance({'vwx', 'vwz', 'omegawy'}, {'dpx', 'dpz', 'nxc', 'nzc'}, 1.1);  

opt = systuneOptions;
opt.Randomstart = 100;
[st, fSoft, gHard] = systune(tfs, [req_distrej], [req_mstab], opt)
K = ss(st.Blocks.K);
Kdisc = c2d(K, postracking.T);
toc

%% Simulate:
x0 = [0 0 0 0 0 1 0 0 ];
sim svm_tracking_1pred_sim;
%sim svm_sim;

%% Debug:
K = (ltiblock.ss('K',Knx,Kny,Knu));
svm_buildTfs;
tfs = [T_mstab] * blkdiag(W_distin, W_distin);
%tfs = minreal(tfs);
opt = hinfstructOptions('Display','final','RandomStart', 50, 'TargetGain', 1);
rng(123);
[Ttuned,gamma,info] = hinfstruct(tfs , opt);
K = ss(Ttuned.Blocks.K)
svm_buildTfs;

%%
tfs = [T_mstab] * blkdiag(W_distin, W_distin);
hinfnorm(Ttuned)
w = [1e-10, 1e2];
opt = sigmaoptions;
opt.MagUnits = 'dB';
sigmaplot(Ttuned, opt);
eig(Ttuned)

%% Have a look at closed loop:
svm_buildTfs;
figure(1);
hold off;
w = {1e-8, 1e4};
subplot(2,1,1);
sigmaplot(T_tracking_xz_slk, w);
lgd = {'tracking error'};
hold on;
grid on;
sigmaplot(T_ceffort_slk, w);
lgd{end+1} = 'control effort';
legend(lgd);

subplot(2,1,2);
hold off;
sigmaplot(W_trackingerrorout, w, 'g*');
hold on;
lgd = {'w_tracking'};
sigmaplot(W_uout, w, 'r*');
lgd{end+1} = 'w_ceffort';
sigmaplot(T_tracking_weighted, w, 'g--');
lgd{end+1} = 'trackingerror_weighted';
sigmaplot(T_ceffort_weighted, w, 'r--');
lgd{end+1} = 'ceffort_weighted';
legend(lgd);
grid on;

hinfnorm(T_tracking_weighted)

%% Preliminaries: estimate achievable closed-loop bandwidth:
clc;
T_x = minreal(uavsim.svm.linearmodels.sorder.airspeed_15(1,1));
T_y = minreal(uavsim.svm.linearmodels.sorder.airspeed_15(2,4));
T_z = minreal(uavsim.svm.linearmodels.sorder.airspeed_15(3,3));

svm_buildTfs;
figure(2);
hold off;
%pzmap(T_x);
pole(T_x)
bode(T_x);
hold on;
bode(T_y);
bode(T_z);
grid on;
lgd = {'Tx', 'Ty', 'Tz'};
legend(lgd);

%% LQR as reference controller:
My = ss(tf([eye(6) -eye(6)]));
plant = uavsim.svm.linearmodels_tss.airspeed_15(1:6, 1:3);
% Add integral states:
plant = addIntegralStates(plant, 1:3);

Q = diag([1 5 1 10 10 10 1 1 1]);
R = 100* diag([1 1 1]);

[Klqr,S,e] = lqr(plant,Q,R)
% Make compatible with Hinf controller:
K = rss(0,Kny,6);
K.d = Klqr;

%% Pseudo-derivative that approximates velocity signal in synthesis models:
s = tf('s');
T_psderivative = s/(1e-6*s + 1);
M_psderivative = ss(blkdiag(T_psderivative, T_psderivative, T_psderivative));
pickP = [eye(3) zeros(3)];
pickV = [zeros(3) eye(3)];
pZeros = zeros(3,6);

% 3 predecessors:
% My = ss(tf([eye(6), zeros(6) zeros(6) -eye(6); ...
%            zeros(6) eye(6) zeros(6) -eye(6); ...
%            zeros(6) zeros(6) eye(6) -eye(6)]));
% 1 predecessor:

My_3D = ([eye(6) -eye(6)]);
% 3 predecessors:
% Mz = ss(tf([pickP, pZeros pZeros -pickP; ...
%            pZeros pZeros pZeros -pickP; ...
%            pZeros pZeros pickP -pickP]));
% 1 predecessor:
Mz = ss(tf([pickP, -pickP]));
% Add integral of position error
Mint = (tf([pickP, -pickP]));
Knx = 0;
Kny = 3;
Knu = 6;
%K = rss(Knx, Kny, Knu);
K = (ltiblock.ss('K',Knx,Kny,Knu));
svm_buildTfs;
tfs = [T_tracking_weighted];
tfs = minreal(tfs);
opt = hinfstructOptions('Display','final','RandomStart', 1, 'TargetGain', 1);
rng(123);
[Ttuned,gamma,info] = hinfstruct(tfs , opt);
K = ss(Ttuned.Blocks.K);


