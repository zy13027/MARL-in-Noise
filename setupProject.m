function setupProject()
% SETUPPROJECT Put the project folders on the MATLAB path.
%   Run once per MATLAB session, from the project root:
%       setupProject
%   Adds src, models, scripts and data to the path, and sends Simulink's
%   generated files (slprj, .slxc) to the work folder so they stay out of
%   the source folders. archive and experiments are left off the path.
root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'src')), ...
        genpath(fullfile(root,'models')), ...
        fullfile(root,'scripts'), ...
        fullfile(root,'data'));

workDir = fullfile(root,'work');
Simulink.fileGenControl('set','CacheFolder',workDir,'CodeGenFolder',workDir,'createDir',true);

fprintf('MARL-in-Noise is on the path. Generated files go to %s\n',workDir);
end
