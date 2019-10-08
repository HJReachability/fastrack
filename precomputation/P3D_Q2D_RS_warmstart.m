function [h, data_no_dist, data_dist, data_warm]=...
    P3D_Q2D_RS_warmstart(gN, visualize)
% 1. First runs reachability with no disturbance (cyan)
% 2. Add disturbance, rerun from scratch (blue)
% 3. Instead of reruning from scratch, maybe we should just initialize with
% the solution from part 1 to "warm start" to the new safety analysis
% (green). Guaranteed to converge to exact solutions if over-optimistic,
% and conservative solutions if over-pessimistic.

% Details: https://arxiv.org/pdf/1903.07715.pdf

if nargin < 1
    % number of grid points (more grid points --> better results, but
    % slower computation)
    gN = [120; 120; 60];
end

if nargin < 2
    visualize = 1;
end

%% Grid and cost

% grid bounds in x, y, theta (relative dynamics)
gMin = [-3.5; -3.5; -pi];
gMax = [ 4.5;  3.5;  pi];

% create grid with 3rd dimension periodic
sD.grid = createGrid(gMin, gMax, gN,3);

% define cost (aka target) function l(x)
% cost is distance to origin (quadratic because derivatives are smoother)
extraArgs.targetFunction = sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2;

% set starting value function equal to cost.
data0 = extraArgs.targetFunction;
% 
% [g2D, data02D] = proj(sD.grid,data0,[0 0 1],'max');
% 
% if visualize
%     figure(1)
%     clf
%     subplot(1,2,1)
%     surf(g2D.xs{1}, g2D.xs{2}, sqrt(data02D))
% end

%% Dynamical system

% tracker control bounds (rotational velocity)
uMin = [-4, -4];
uMax = [4, 4];

% planner control bounds (velocity in x and y)
pMax = [.1, .1];
pMin = [-.1, -.1];

% disturbance control bounds (velocity in x and y)
dMax = [0; 0];
dMin = [0; 0];

% create relative dynamics
%P3D_Q2D_Rel(x, uMin, uMax, pMin, pMax, dMin, dMax, v, dims)
sD.dynSys = P3D_Q2D_Rel([], uMin, uMax, pMin, pMax, dMin, dMax);

%% Otherparameters
% tracker trying to minimize, disturbances (including planner) trying to
% maximize
sD.uMode = 'min';
sD.dMode = 'max';

% how carefully are we measuring gradients?
sD.accuracy = 'medium';

if visualize
    
    % Viewing a slice of the value function
    % Can instead view set if you prefer
    extraArgs.visualize.valueFunction = 1;
    
    % Viewing X,Y slice
    extraArgs.visualize.plotData.plotDims = [1 1 0];
    
    % Projecting theta to 0 (car facing towards the positive x axis)
    % can also plot at "min" or "max" to do union/intersection
    extraArgs.visualize.plotData.projpt = 0;
    
    % View Axis
    extraArgs.visualize.viewAxis = ...
        [gMin(1) gMax(1) gMin(2) gMax(2) 0 10];

    % Figure number
    extraArgs.visualize.figNum = 2;
    
    % delete the previous plot over time
    extraArgs.visualize.deleteLastPlot = 1;
    
    % Print convergence over time in title
    extraArgs.visualize.convergeTitle = 1;
    
    % Color of value function
    extraArgs.visualize.plotColorVF = 'c';
end
% code will visualize and check convergence at every dt seconds
dt = 0.1;

% upper bound on how long the code can keep going (hopefully converges far
% before this time)
tMax = 100;

% time stamps
tau = 0:dt:tMax;

% stop if convergence is reached
extraArgs.stopConverge = true;

% convergence threshold
extraArgs.convergeThreshold = dt;

% only keep the most recently computed data
extraArgs.keepLast = 1;

%% Run Original Analysis W/O Disturbance (Cyan)
tic
% in FaSTrack we want to take the max with the cost function l(x) over time
compMethod = 'maxVWithL';

% The main function to run reachability code
[data_no_dist, tau, extraOuts] = ...
    HJIPDE_solve(data0, tau, sD, compMethod, extraArgs);

runtime.original = toc;

% grab the figure handle
h.original = extraOuts.hVF;

%% What hold on there are disturbances, better recompute everything 
% from scratch! (blue)

% change disturbance bounds
dMax = [0.2; 0.2];
dMin = [-0.2; -0.2];

% update dynamic system with new disturbance bounds
sD.dynSys = P3D_Q2D_Rel([], uMin, uMax, pMin, pMax, dMin, dMax);

if visualize
    extraArgs.visualize.holdOn = 1;
    extraArgs.visualize.plotColorVF = 'b';
end

tic

% run from scratch with new dynamic system (with new disturbances)
[data_dist, tau, extraOuts] = ...
    HJIPDE_solve(data0, tau, sD, compMethod, extraArgs);

runtime.new = toc;
h.new = extraOuts.hVF;

%% Don't do that dummy, we can warm-start with the original value function 
% we computed! (green)

if visualize
    extraArgs.visualize.plotColorVF = 'g';
end

tic

% dynamic system is still the one with newly added disturbances. Only
% change is now we're going to initialize with the output to the orignal
% computation (data_no_dist) instead of data0
[data_warm, tau, extraOuts] = ...
    HJIPDE_solve(data_no_dist, tau, sD, compMethod, extraArgs);
runtime.warm = toc;
h.warm = extraOuts.hVF;

end
