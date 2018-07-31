% RunFtmPE - main positioning engine loop.
% usage:[posEst,pValid] = RunFtmPE(cfg, measTable)

% For questions/comments contact: 
% leor.banin@intel.com, 
% ofer.bar-shalom@intel.com, 
% nir.dvorecki@intel.com,
% yuval.amizur@intel.com

% Copyright (C) 2018 Intel Corporation
% SPDX-License-Identifier: BSD-3-Clause

function [posEst,pValid,bias,latErrPredict,latErrUpdate,KFtime] = RunFtmPE(cfg, measTable)

measN = size(measTable,1);
KF = KfFtmclass(cfg); % Kalman Filter Constructor
% KF = CalcPositionClass(cfg); % Kalman Filter Constructor
INIT_KF = 1;  % KF Initialization flag

posEst = nan(3,measN);
pValid = false(measN,1);
updateDone = false(measN,1);
bias = nan(measN,1);
latErrPredict = nan(measN,1);
latErrUpdate = nan(measN,1);
KFtime = nan(measN,1);

k = 1;
while (k <= measN)
    if INIT_KF
        timestamp = measTable(1,1);
        EstPos =  measTable(1,2:4)'; % init. value to ground-truth
        KF.initKF(timestamp,EstPos);
        INIT_KF = 0; % Reset flag
        continue;
    else
        [timestamp,measRange,Rsp] = ParseMeasLine(measTable(k,:));
        % Run KF
        if(measRange <= cfg.MaxRangeFilter) 
            [posEst(:,k),updateDone(k),bias(k),latErrPredict(k),latErrUpdate(k),KFtime(k)] = ...
                KF.Run(measRange,timestamp,Rsp); 
            pValid(k) = true;
        else
            pValid(k) = false;
        end
    end % INIT_KF
    k = k + 1;
end % while
end % function

% ------------------------------------------------------------------------
function [timestamp,measRange,Rsp] = ParseMeasLine(measLine)

% line Format: timestamp,rspId,measRange,rangeSigma,rspPosX,rspPosY,rspPosZ
timestamp      = measLine(1);
% refPos         = measLine(2:4)';
measRange      = measLine(5);
%----------------------------------------------------------
Rsp.pos        = measLine(6:8)';   % Transmitting responder position
Rsp.index      = measLine(9);      % Responder ID
%----------------------------------------------------------
end

