% Sets up optimization environment for acceleration and parallel computing
% Copyright 2017-2019 The MathWorks, Inc.

if parallelFlag
    
    % Create a parallel pool if one is not active
    if isempty(gcp('nocreate'))
        parpool; % Uses default saved profile
    else
        delete(gcp);
        parpool;
    end
    p = gcp;
    
    % Prevent parallel simulations from accessing Simulation Data Inspector
    Simulink.sdi.enablePCTSupport('all');
    
    % Add paths and dependent files to run simulations in parallel
    rootDir = fullfile(fileparts(mfilename('fullpath')),'');
    addAttachedFiles(p,rootDir);
    parfevalOnAll(@addpath,0,rootDir);
    parfevalOnAll(@load_system,0,mdlName);
    
    % If the acceleration flag is true, set the simulation mode to
    % accelerator
    if accelFlag
        parfevalOnAll(@set_param,0,mdlName,'SimulationMode','accelerator');
        parfevalOnAll(@set_param,0,mdlName,'SimMechanicsOpenEditorOnUpdate','off');
    end
    
    % Change each worker to unique folder so cache files do not conflict
    if exist('temp','dir')
        rmdir('temp','s');
    end
    mkdir('temp');
    spmd
        tempFolder = fullfile(rootDir,'temp');
        cd(tempFolder);
        folderName = tempname(tempFolder);
        mkdir(folderName);
        cd(folderName)
    end
    
else
   % If the acceleration flag is true, set the simulation mode to
   % accelerator
   if accelFlag
        load_system(mdlName);
        set_param(mdlName,'SimulationMode','accelerator');
        set_param(mdlName,'SimMechanicsOpenEditorOnUpdate','off');
   end
end