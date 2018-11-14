% ReadFtmMeasFile - extracts FTM measurement file.
% usage: [measTableOut,refPosMat,rPos] = ReadFtmMeasFile(cfg)

% For questions/comments contact: 
% leor.banin@intel.com, 
% ofer.bar-shalom@intel.com, 
% nir.dvorecki@intel.com,
% yuval.amizur@intel.com

% Copyright (C) 2018 Intel Corporation
% SPDX-License-Identifier: BSD-3-Clause

% function [measTableOut,rPos,refPosMat] = ReadFtmMeasFile(cfg)
function [measTableOut,refPosMat,rPos,VenueParams] = ReadFtmMeasFile(cfg)


measTable    = load(fullfile(pwd,'\data\',cfg.sessionFolder,cfg.measFile));
rPos         = load(fullfile(pwd,'\data\',cfg.sessionFolder,cfg.rPosFile));
VenueParams  = load(fullfile(pwd,'\data\',cfg.sessionFolder,cfg.VenueFile));

Nrsp = length(rPos(:,1));
% remove RSPs ---------------------------------------------------
for k = 1:length(cfg.Rsp2remove)
    id = cfg.Rsp2remove(k);
    if(id <= Nrsp)
        z  = measTable(:,9)==id;
        measTable(z,:)=[];
    end
end
rPos(cfg.Rsp2remove,:) = [];
% ----------------------------------------------------------------
% extract reference trajectory - ground truth of client position
refPosMat = measTable(:,2:4); 

% prepare input to PE
measTableOut = measTable;