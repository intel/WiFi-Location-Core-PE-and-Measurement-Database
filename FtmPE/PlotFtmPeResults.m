% PlotResults displays estimated and reference trajectories on venue map.
% In addition, it plots the empirical cumulative distribution function
% Usage: PlotResults(cfg,posMat,bPos,refPos)

% For questions/comments contact: 
% leor.banin@intel.com, 
% ofer.bar-shalom@intel.com, 
% nir.dvorecki@intel.com,
% yuval.amizur@intel.com

% Copyright (C) 2018 Intel Corporation
% SPDX-License-Identifier: BSD-3-Clause

function PlotFtmPeResults(cfg,posMat,rPos,refPos,bias, latErrPredict, latErrUpdate, venueParams,timeVec)

figure; 
hAxes=gca;
cfg.name(cfg.name=='_')=' ';

if ~isempty(hAxes)
    imagesc(venueParams.cfgVenue.xMinMax,venueParams.cfgVenue.yMinMax,venueParams.cfgVenue.I,'parent',hAxes)
    set(hAxes,'ydir','normal');
    axis(hAxes,'equal');
    hold(hAxes,'on');
    plot(refPos(:,1),refPos(:,2),'r.')
    plot(posMat(:,1),posMat(:,2),'b.')
    plot(rPos(:,2),rPos(:,3),'m^','MarkerFaceColor','m')
    leg1 = legend('$\mathbf{p}$','$\hat{\mathbf{p}}$','RSP','Location','southwest');
    set(leg1,'Interpreter','latex');
    for k = 1:size(rPos,1)
        text(rPos(k,2)+1,rPos(k,3),['\bf', num2str(rPos(k,1))],'FontSize',16,'Color','r');
    end
end
xlabel('[m]')
ylabel('[m]')
axis equal
axis([-45 7 -5 25]) 
title(cfg.name)

tmp = (posMat-refPos);
product = tmp.*tmp;
error =  sqrt(squeeze(sum(product,2)));

figure
subplot 321
plot(timeVec-timeVec(1),posMat(:,3),'b','LineWidth',1)
xlabel('time [sec]')
ylabel('z height[m]')
grid on; box on;
subplot 323
plot(timeVec-timeVec(1),bias,'k','LineWidth',1)
xlabel('time [sec]')
ylabel('Bias [m]')
grid on; box on;
subplot 325
plot(timeVec-timeVec(1),error./latErrPredict,'m','LineWidth',1)

xlabel('time [sec]')
ylabel('\sigma')
grid on; box on;
subplot(3,2,[2,4,6])
[X1,Y1] = CalcEcdf(error./latErrPredict);
plot(X1,Y1,'r-','LineWidth',2)
ylabel('CDF[%]')
xlabel('\sigma')
grid on

text('VerticalAlignment','top',...
    'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'String',cfg.name,...
    'Position',[-0.46 107.4 0],...
    'Visible','on');

[X,Y] = CalcEcdf(error);
figure
plot(X,Y,'-','LineWidth',2)
xlabel('Position Error [m]')
ylabel('CDF [%]')
title(cfg.name)
grid on
box on
end

% -----------------------------------------------------
function [X,Y] = CalcEcdf(in)
N = length(in);
in_s = sort(in);
Y = (1:N)/N*100;
X=in_s(:);Y=Y(:);
end

