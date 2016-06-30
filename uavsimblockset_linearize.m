% Computes linearized models e.g. for control design. The simulink model
% should have the desired 12 states as outputs 1...12 and the desired outputs as
% outputs 13....(12+ny).
% Authors: Jan Bolting, Martin Stolle
% email address: jan.bolting@gmx.de; martin.stolle@mail.com
% March 2014; Last revision: 28-March-2014

function [linearModel, newtrim, imap] = uavsimblockset_linearize(trimModel, xTrim, uTrim, yTrim, ix)
% Obtain linear model with states choosen automagically by simulink
% (velocity_body eulers omega position):
linearizedTrimModel = linmod(trimModel, xTrim, uTrim);
ssm = ss(linearizedTrimModel.a, ...
    linearizedTrimModel.b, ...
    linearizedTrimModel.c, ...
    linearizedTrimModel.d);
% Build new system with the desired states and the desired outputs:
% 1. Intermediate: Build new system with the desired states (keep same
%    outputs):
% Select new states out of old output:
% Re-order outputs, new states on top:
top = linearizedTrimModel.c(ix, :);
ibottom = 1:length(yTrim);
ibottom(ix) = [];
bottom = linearizedTrimModel.c(ibottom, :);
C = [top; bottom];
% Calculate transformation matrix and convert to the new system:
C_x_bar = C(1:12, 1:12);
linearizedTrimModel_newStates = ss2ss(ssm, C_x_bar);

% 2. Intermediate: Set new outputs
A_new = linearizedTrimModel_newStates.a;
B_new = linearizedTrimModel_newStates.b;
C_y_bar = linearizedTrimModel.c;
C_new = C_y_bar* inv(C_x_bar);
D_new = linearizedTrimModel.d(:,:);
linearizedTrimModel_newStatesAndOutputs = ss(A_new, B_new, C_new, D_new);

% 3. Update variable names
% Get output and inport port labels of simulink model:
load_system(trimModel)
labels_out = find_system(trimModel,'SearchDepth',1,'BlockType','Outport');
labels_in = find_system(trimModel,'SearchDepth',1,'BlockType','Inport');
% Only use last element as signal name:
outnames = {};
% Build map of names->indices to ease access:
imap.x = containers.Map();
imap.y = containers.Map();
imap.u = containers.Map();

for i=1:length(labels_out)
    C = strsplit(labels_out{i}, '/');
    outnames{end+1} = C{end};
    imap.y(C{end}) = i;
end
innames = {};
for i=1:length(labels_in)
    C = strsplit(labels_in{i}, '/');
    innames{end+1} = C{end};
    imap.u(C{end}) = i;
end

linearizedTrimModel_newStatesAndOutputs.StateName(:, 1) = outnames(ix);
linearizedTrimModel_newStatesAndOutputs.OutputName(:, 1) = outnames;
linearizedTrimModel_newStatesAndOutputs.InputName(:, 1) = innames;

for i=1:length(linearizedTrimModel_newStatesAndOutputs.StateName)
    imap.x(linearizedTrimModel_newStatesAndOutputs.StateName{i}) = i;
end

% Update trim:
newtrim.uTrim = uTrim;
newtrim.yTrim = yTrim;
newtrim.xTrim = yTrim(ix);

% Resulting model:
linearModel = linearizedTrimModel_newStatesAndOutputs;
% Remove small elements that are merely numerical artifacts:
linearModel.a = fflib_removeSmallElements(linearModel.a, 1e-6);
linearModel.b = fflib_removeSmallElements(linearModel.b, 1e-6);
linearModel.c = fflib_removeSmallElements(linearModel.c, 1e-6);
linearModel.d = fflib_removeSmallElements(linearModel.d, 1e-6);

save_system(trimModel);
close_system(trimModel);
display([mfilename '>> Extracted linear models from ' trimModel]);
end
