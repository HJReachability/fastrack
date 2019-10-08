function [datas,tau,sD,trackingErrorBound]=Q6D_Q3D_RS(pMax, thrustRange, angleRange, gN, visualize)

small = .3449;%0.1414;
dims = 1:6;
subDims = {[dims(1),dims(2)], [dims(3),dims(4)],[dims(5),dims(6)]};
subDimNames = ['x','y','z'];

if nargin <1
    pMax = .6;
end

if nargin <2
    thrustRange = [9.8-2 9.8+2];
end

if nargin <3
    angleRange = [-.1 .1];% in radians
end

if nargin < 4
    gN = 275*ones(1,length(dims));
end

if nargin < 5
    visualize = 1;
end

%% Dynamical system

% min and max controls for the tracker
%angleRangeRad = deg2rad(angleRange);
angleRangeRad = angleRange;
uMin = [angleRangeRad(1); angleRangeRad(1); thrustRange(1)];
uMax = [angleRangeRad(2); angleRangeRad(2); thrustRange(2)];

% min and max velocities for planner (in x and y)

min_planner_speed = -pMax*ones(length(subDims),1);
max_planner_speed = pMax*ones(length(subDims),1);

% min and max disturbance velocity
dRangeV = [-max(.2,pMax/2); max(.2,pMax/2)];

% min and max disturbance acceleration
dRangeA = [-.1; .1];

dMin = [dRangeV(1)*ones(length(subDims),1); ...
    dRangeA(1)*ones(length(subDims),1)];
dMax = [dRangeV(2)*ones(length(subDims),1); ...
    dRangeA(2)*ones(length(subDims),1)];

% create dynamic systems
sD = cell(1,length(subDims));
for ii = 1:length(subDims)
    sD{ii}.dynSys = Q6D_Q3D_Rel([], uMin, uMax, dMin, dMax, ...
        min_planner_speed, max_planner_speed, subDims{ii});
end

%% Grid and cost

% Grid Bounds
gMin = -5*ones(1,length(dims));
gMax = 5*ones(1,length(dims));

% createGrid takes in grid bounds, grid number, and periodic dimensions
for ii = 1:length(subDims)
    sD{ii}.grid = createGrid(gMin(subDims{ii}), gMax(subDims{ii}), gN(subDims{ii}));
end

% Cost Function
data0 = cell(1,length(subDims));

% make cost dependent on position states in each subsystem
pdim = cell(1,length(subDims));
for ii = 1:length(subDims)
    pdim{ii} = intersect(subDims{ii},sD{ii}.dynSys.pdim);
    if ~isempty(pdim{ii})
        idx = find(subDims{ii}==pdim{ii}(1));
        data0{ii} = sD{ii}.grid.xs{idx}.^2;
    end
    for jj = 2:length(pdim{ii})
        idxNext = find(subDims{ii}==pdim{ii}(jj));
        data0{ii} = data0{ii} + sD{ii}.grid.xs{idxNext}.^2;
    end
end

if visualize
    level = 1;
    figure(1)
    clf
    hInit = cell(1,length(subDims));
    for ii = 1:length(subDims)
        subplot(1,3,ii)
        hInit{ii} = visSetIm(sD{ii}.grid, data0{ii}, 'b', level);
    end
end



%% Other Parameters

for ii = 1:length(subDims)
    % is tracker minimizing or maximizing?
    sD{ii}.uMode = 'min';
    
    % is planner minimizing or maximizing?
    sD{ii}.dMode = 'max';
    
    % how high accuracy?
    sD{ii}.accuracy = 'veryHigh';
end

if visualize
    % set visualize to true
    extraArgs.visualize.valueFunction = 1;
    
    % set slice of function to visualize
    extraArgs.visualize.sliceLevel = level;
    
    % figure number
    extraArgs.visualize.figNum = 2;
    f = figure(2);
    %   set(f, 'Position', [400 400 450 400]);
    
    % delete previous time step's plot
    extraArgs.visualize.deleteLastPlot = true;
end

% time step
dt = 0.01;

% Max time
tMax = 10;

% Vector of times
tau = 0:dt:tMax;

% stop when function has converged
extraArgs.stopConverge = true;

% converges when function doesn't change by more than dt each time step
extraArgs.convergeThreshold = dt/2;

extraArgs.keepLast = 1;

extraArgs.quiet = 0;

% solve backwards reachable set
datas = cell(1,length(subDims));
for ii = 1:length(subDims)
    [datas{ii}, ~] = HJIPDE_solve(data0{ii}, tau, ...
        sD{ii}, 'maxVWithV0', extraArgs);
end

trackingErrorBound = zeros(1,3);
for ii = 1:length(subDims)
    trackingErrorBound(ii) = min(sqrt(datas{ii}(:)))+small;
    insideBound{ii} = datas{ii}(datas{ii}<=trackingErrorBound(ii));
    q{ii} = histogram(insideBound{ii}(:));
    mass50 = sum(q{ii}.Values(:))/2;
    mass95 = .95*sum(q{ii}.Values(:));
    lower_test = 0;
    upper_test = 0;
    bin = 1;
    while lower_test < mass50
        lower_test = lower_test + q{ii}.Values(bin);
        bin = bin + 1;
    end
    priority_lower_bound{ii} = q{ii}.BinEdges(bin);
    
    bin = 1;
    while upper_test < mass95
        upper_test = upper_test + q{ii}.Values(bin);
        bin = bin + 1;
    end
    priority_upper_bound{ii} = q{ii}.BinEdges(bin);
end

if visualize
    figure(3)
    clf
    h = cell(1,length(subDims));
    h0 = cell(1,length(subDims));
    for ii = 1:length(subDims)
        subplot(1,3,ii)
        h{ii} = visSetIm(sD{ii}.grid, sqrt(datas{ii}), 'r', trackingErrorBound(ii));
        hold on
        h0{ii} = visSetIm(sD{ii}.grid, sqrt(data0{ii}), 'b', trackingErrorBound(ii));
    end
end

matlabFolder = '/Users/sylvia/Documents/MATLAB';
plannerFolder = sprintf('%s/planner_RRT3D', matlabFolder);
if ~exist(plannerFolder, 'dir')
  mkdir(plannerFolder);
end

speedFolder = [plannerFolder '/speed_' ...
    num2str(pMax*10) '_tenths'];
if ~exist(speedFolder, 'dir')
  mkdir(speedFolder);
end

plannerFolderMatlab = sprintf('%s/planner_RRT3D_Matlab', matlabFolder);
if ~exist(plannerFolderMatlab, 'dir')
  mkdir(plannerFolderMatlab);
end

for ii = 1:length(sD)
    datas{ii} = sqrt(datas{ii});
    derivs{ii} = computeGradients(sD{ii}.grid,datas{ii});
end


for ii = 1:length(subDims)
    data = datas{ii};
    grid_min = gMin(subDims{ii})';
    grid_max = gMax(subDims{ii})';
    grid_N = uint64(gN(subDims{ii}))';
    teb = zeros(length(subDims{ii}),1);
    idx = find(subDims{ii}==pdim{ii});
    teb(idx) = trackingErrorBound(ii);
    x_dims = uint64(subDims{ii}-1)'; %0-index
    u_dims = (uint64(ii)-1)';
    u_min = uMin(ii)';
    u_max = uMax(ii)';
    d_min = sD{1}.dynSys.dMin(subDims{1});
    d_max = sD{1}.dynSys.dMax(subDims{1});
    deriv0 = derivs{ii}{1};
    deriv1 = derivs{ii}{2};
    priority_lower = priority_lower_bound{ii};
    priority_upper = priority_upper_bound{ii};
    % note DON'T use -v7.3 extension for compression. it fucks up the C++
    % stuff
    save([speedFolder '/subsystem_' subDimNames(ii) '.mat'], ...
        'data','grid_min','grid_max','grid_N','teb','x_dims','u_dims',...
        'u_min','u_max', 'max_planner_speed','deriv0', 'deriv1', ...
        'priority_lower','priority_upper')
end

if ~exist([plannerFolderMatlab '/speed_' ...
    num2str(pMax*10) '_tenths.mat'], 'file')
      save([plannerFolderMatlab '/speed_' ...
    num2str(pMax*10) '_tenths.mat'], ...
        'datas','tau','sD','trackingErrorBound','derivs', ...
        'priority_lower_bound','priority_upper_bound');
end
end

