function [nc, sigma, dbport ] = dsmc_extrinsic1(dp, dv, da, tsim)
persistent history;
persistent T;
persistent kd;
% Detect first sample, and initialize persistent variables on it:
if tsim < eps
    T = 0;
    kd = 0;
    T = evalin('base', 'postracking.T');
    td = evalin('base', 'svm.delays.input');
    kd = fflib_delay_c2d(td, T);
    % history(k) means the sample is k samples in the past, i.e. history(1)
    % is the sample before the current one.
    history(1).sigma = zeros(3,1);
    history(1).u_apst = zeros(3,1);
    history(1).Phi_k = zeros(3,1);
    history(1).x = zeros(6,1);
    history(1).da = zeros(3,1);
    history(1).dv = zeros(3,1);
    history(1).dp = zeros(3,1);
    history(1).nc = zeros(3,1);
    % Save at least 2 samples to not break the code:
    history(2) = history(1);
    % If there is an input delay, expand the sample history to cover the
    % whole delay:
    for p=1:kd
        history(p) = history(1);
    end
end
u = zeros(3,1);
sigma = zeros(3,1);

R_eb = eye(3);
R_be = R_eb';
nc_0 = R_eb*[0 0 -1]';
x = [dp; dv];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State predictor to compensate for input delays:
A = [eye(3)   T*eye(3);
    zeros(3) eye(3)];
B = [zeros(3); T*eye(3)];
% Estimate lumped acceleleration perturbations during the transition from
% the last sample to the current one:
a_w = dv - history(1).dv - T*history(1).nc*9.81;
x_tilde = x;
for k = 1:kd
    % The control input that arrives at the plant at the sample k is the
    % control input of (k-kd):
    a_c = history(kd-k+1).nc*9.81;
    % Due to lack of data, assume that perturbations stay the same:
    x_tilde = A*x_tilde + B*(a_c + a_w);
end
% Disable predictor:
%x_tilde = x;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DSMC:
Psi = 0.93*eye(3);
usat_upper = zeros(3,1);
usat_lower = zeros(3,1);
usat_upper = evalin('base', 'uavsim.svm.usat.n_c.upper');
usat_lower = evalin('base', 'uavsim.svm.usat.n_c.lower');
lp = [6 6 6];
lv = [4 4 4];
G = [diag(lp) diag(lv)];
sigma = G * x_tilde;
B = [zeros(3); eye(3)];
deltapc_ddot = zeros(3,1);
Phi_k = G*[dv; -deltapc_ddot];
Phi_u_tilde = sigma ...
    - history(1).sigma ...
    - T*(history(1).Phi_k ...
    + history(1).u_apst);
u_apst = (1/T)*(Psi-eye(3))*sigma - Phi_k - Phi_u_tilde;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PDSMC:
problem.options = optimoptions('fmincon');
problem.options.Display = 'off';
problem.options.MaxIter = 100;
%problem.options.OutputFcn = @callmelongtime;
problem.objective = @cf;
du_apst_max = 100*[1 1 1]';
% Saturations on u_apst:
u_apst_upper = G*B*R_eb*9.81*(usat_upper - nc_0);
u_apst_lower = G*B*R_eb*9.81*(usat_lower - nc_0);
uoptim_upper = sat(history(1).u_apst + du_apst_max, u_apst_lower, u_apst_upper);
uoptim_lower = sat(history(1).u_apst - du_apst_max, u_apst_lower, u_apst_upper);
problem.lb = uoptim_lower;
problem.ub = uoptim_upper;
problem.x0 = u_apst;
problem.solver = 'fmincon';
c = T*(Phi_k + Phi_u_tilde);
%[uoptim, fval] = fmincon(problem);
%u_apst = uoptim - c;

%%%%%%%%
% PDSCM as quadratic program:
Px = eye(3);
Hx = T*eye(3);
Q = eye(3);
% Set up QP problem:
H = Hx*Q*Hx;
f = sigma'*Px*Q*Hx;
A = [];
b = [];
Aeq = [];
beq = [];
lb = uoptim_lower;
ub = uoptim_upper;
x0 = history(1).u_apst;
opt = optimoptions('quadprog');
opt.Display = 'off';
tic;
[uoptim,fval,exitflag,output] = quadprog(H,f,A,b,Aeq,beq,lb,ub, x0, opt);
dbport = toc;
u_apst = uoptim - c;
x;
sigma;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute commanded load factors:
% First saturate u_apst in case of unconstrained law:
u_apst = sat(u_apst, u_apst_lower, u_apst_upper);
% Then compute corresponding load factors plus trim input:
nc = (1/9.81)*eye(3)*(G*B)^-1*u_apst + nc_0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Shift sample history by one and save variables of this time step:
history(2:end) = history(1:end-1);
history(1).sigma = sigma;
history(1).u_apst = u_apst;
history(1).nc = nc;
history(1).Phi_k = Phi_k;
history(1).da = da;
history(1).dv = dv;
history(1).dp = dp;
history(1).x = x;

    function cost = cf(upot)
        sigma_pred = sigma + T*(upot);
        M = eye(3);
        cost = sigma_pred'*M*sigma_pred; %sum(abs(sigma_pred));
    end

end