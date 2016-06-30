save('uavsimblockset_environment.mat');

% Store all relevant variables necessary to run uavsimblockset code on
% another machine.
% models = {'svm_sim_singleUAS', 'svm_sim_multipleUAS', 'svm_test', 'Cularis_nxz', 'Cularis_openloop', 'svm_sim_multipleUAS_shapewarping'};
% vars = [];
% for k=1:length(models)
%     load_system(models{k});
%     vars =  [vars; Simulink.findVars(models{k}, 'SourceType', 'base workspace')];
% end
% save('uavsimblockset_environment.mat', vars.Name);