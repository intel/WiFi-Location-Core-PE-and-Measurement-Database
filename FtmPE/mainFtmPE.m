% mainPE - main function for simulation.
% usage: mainPE(configMfile)

% For questions/comments contact: 
% leor.banin@intel.com, 
% ofer.bar-shalom@intel.com, 
% nir.dvorecki@intel.com,
% yuval.amizur@intel.com

% Copyright (C) 2018 Intel Corporation
% SPDX-License-Identifier: BSD-3-Clause

function mainFtmPE(sessionFolder)

if  exist('sessionFolder','var')
    cfg = testFtmPeConfig(sessionFolder);
else
    cfg = testFtmPeConfig();
end

[measTable,refPos,rPos,venueParams]                       = ReadFtmMeasFile(cfg);    % Read measurements file
[posEst,pValid,bias,latErrPredict,latErrUpdate,timeVec]   = RunFtmPE(cfg,measTable); % Run PE

% pValid - designates the entries produced for every measurement received by the client

posMat        = posEst(:,pValid)';
refPosMat     = refPos(pValid,:);
bias          = bias(pValid,:);
latErrPredict = latErrPredict(pValid,:);
latErrUpdate  = latErrUpdate(pValid,:);
timeVec       = timeVec(pValid,:);
PlotFtmPeResults(cfg,posMat,rPos,refPosMat,bias, latErrPredict, latErrUpdate, venueParams, timeVec) % plot results

end
