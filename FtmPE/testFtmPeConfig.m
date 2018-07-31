% For questions/comments contact: 
% leor.banin@intel.com, 
% ofer.bar-shalom@intel.com, 
% nir.dvorecki@intel.com,
% yuval.amizur@intel.com
% Copyright (C) 2018 Intel Corporation
% SPDX-License-Identifier: BSD-3-Clause

function cfg = testFtmPeConfig(sessionFolder)
if  ~nargin
    cfg.sessionFolder = 'Office1_80MHz';
else
    cfg.sessionFolder = sessionFolder;
end
cfg.name                  = cfg.sessionFolder;
cfg.measFile              = [cfg.name,'.csv'];
cfg.rPosFile              = [cfg.name,'_RSP_LIST.csv'];
cfg.VenueFile             = [cfg.name,'_VenueFile.mat'];
%*************************************************************************
cfg.UseSyntheticMeas      = 1; % 1 = Use synthetic measurements
                               % 0 = Use real measured data
if cfg.UseSyntheticMeas
    cfg.measFile = [cfg.name,'_noisySynthRanges.csv'];
    cfg.name = cfg.measFile(1:end-4);
end
%*************************************************************************    
cfg.Rsp2remove            = [];% set e.g., cfg.Rsp2remove = [1,6] to remove RSPs {1,6} 
cfg.scaleSigmaForBigRange = 1; % 1 = Enable STD scaling for range, otherwise set to 0 
cfg.outlierFilterEnable   = 1; % 1 = Enable Outlier Filtering, otherwise set to 0
cfg.MaxRangeFilter        = 50;  % filter out ranges above this threshold
cfg.OutlierRangeFilter    = 30;  % enable outlier range filtering above this threshold
cfg.gainLimit             = 3;   % EKF gain limit
%*************************************************************************                               

cfg.knownZ                = 1.4; % Known client height [meter]
cfg.rangeMeasNoiseStd     = 1.0; % Range measurement noise Std [meter]. 
cfg.zMeasNoiseStd         = 0.1; % Height measurement noise Std [meter].

cfg.posLatStd             = 1.0; % Q - sys. noise [meter per second];
cfg.heightStd             = 0.1; % Q - sys. noise [meter per second];
cfg.biasStd               = 0.01;% Q - sys. noise [meter per second];
cfg.init.posLatStd        = 1;   % P - state cov. [meter] 
cfg.init.heightStd        = 0.2; % P - state cov. [meter] 
cfg.init.biasStd          = 0.5; % P - state cov. [meter] 

% measurement type defines
cfg.MEAS_RANG             = 1;
cfg.MEAS_CONST_Z          = 2;


end