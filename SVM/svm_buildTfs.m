% Longitudinal:
% Extract open-loop model from Simulink.
M = linearize('svm_tracking_1pred');
% Build mesh stability transfer:
G = linearize('svm_mstab');
% Connect controllers to UAS 1 and UAS 2:
Ks = blkdiag(K, K);
G = lft(G, Ks);
T_mstab = G(1:2, 1:2);

%Connect to controller:
P = lft(M, K);
% Add feed-forward:
% Tracking longi:
T_tracking_x_slk = P([1], [1]);
T_tracking_z_slk = P([2], [2]);
T_tracking_xz_slk = blkdiag(T_tracking_x_slk, T_tracking_z_slk);
%T_tracking_xz_slk = P([1 2], [1 2]);
% Control effort:
T_ceffort__tracking_slk = P(3:4, 1:2);
% Decoupling between x and z:
T_decoupling_x2z_slk = P(2, 1);
T_decoupling_z2x_slk = P(1, 2);
T_decoupling_slk = blkdiag(T_decoupling_x2z_slk, T_decoupling_z2x_slk);

% Disturbance rejection:
T_dist_slk = P(1:2, 3:5);
% Disturbances to control effort:
T_ceffort_dist_slk = P(3:4, 3:5);

% T = slTunable('svm_tracking_1pred', {'K'});
% %T_tracking_slk = T.getIOTransfer({'p_pre_1', 'p_pre_2', 'p_pre_3'}, 'delta_p1');
% T_tracking_slk = T.getIOTransfer({'p_pre_1'}, 'delta_p');
% %T_ceffort_slk = T.getIOTransfer({'p_pre_1', 'p_pre_2', 'p_pre_3'}, 'u');
% T_ceffort_slk = T.getIOTransfer({'p_pre_1'}, 'u');

% Weighting and scaling of tracking error:
%W_trackingerrorout = fflib_build2ndOrderLowpass(5, 1, 1)/0.1;
W_refin = fflib_build2ndOrderLowpass(10, 1, 1);
W_distin = fflib_build2ndOrderLowpass(100, 1, 1);
% Weighting and scaling of control effort:
W_uout = fflib_build2ndOrderHighpass(50, 1, 1);

W_z = blkdiag(W_refin, W_refin);