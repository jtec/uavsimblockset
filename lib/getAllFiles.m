% Recursively gets all files and directories below a directory, filters for file
% extensions.
function [files, subDirs ]= getAllFiles(dirName, ext)

dirData = dir(dirName);      %# Get the data for the current directory
dirIndex = [dirData.isdir];  %# Find the index for directories
fileList = {dirData(~dirIndex).name}';  %'# Get a list of the files
if ~isempty(fileList)
    fileList = cellfun(@(x) fullfile(dirName,x),...  %# Build full file path
        fileList,'UniformOutput',false);
end
% Get subdirectories:
subDirs = {dirData(dirIndex).name}';  %'# Get a list of the directories
% Find index of subdirectories that are not '.' or '..':
validIndex = ~ismember(subDirs,{'.','..'});
subDirs = subDirs(find(validIndex));
if ~isempty(subDirs)
    subDirs = cellfun(@(x) fullfile(dirName,x),...  %# Build full directory path
        subDirs,'UniformOutput',false);
end

for iDir = 1:length(subDirs)                  %# Loop over valid subdirectories
    nextDir = subDirs{iDir};    %# Get the subdirectory path
    % Recursively call getAllFiles
    [fs ds] = getAllFiles(nextDir, ext);
    fileList = [fileList; fs];
    subDirs = [subDirs; ds];
end
% Filter for desired file extension:
files = [];
if ~isempty(fileList)
    for iFile=1:length(fileList)
        [fPath, fName, fExt] = fileparts(fileList{iFile, 1});
        % If the desired file extension is empty, all files are
        % accepted:
        if isempty(ext)
            files{end+1, 1} = fileList{iFile, 1};
        else
            if strcmp(fExt,ext) == 1
                files{end+1, 1} = fileList{iFile, 1};
            end
        end
    end
    a = ext;
    
end
return;
end