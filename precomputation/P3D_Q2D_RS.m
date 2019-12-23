function [data,tau,sD,TEB]=P3D_Q2D_RS(gN, visualize)
% 3D "plane" (dubins car) tracking 2D point mass

% default to more grid points in x and y than in theta
if nargin < 1
    % number of grid points (more grid points --> better results, but
    % slower computation)
    gN = [80; 80; 50];
end

% default to visualizing
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

if visualize
    figure(1)
    clf
    subplot(1,2,1)
    % project down to 2D so we can see the value function
    [g2D, data02D] = proj(sD.grid,data0,[0 0 1],'min');
    
    
    color = 'b'; % color
    alpha = .75; % transparency
    
    % visualize sqrt of cost
    hF0 = visFuncIm(g2D, sqrt(data02D),color,alpha);
    
    % make pretty
    view(30,10)
    c = camlight;
    c.Position = [4 -2 1];
    axis([gMin(1) gMax(1) gMin(2) gMax(2) 0 3])
    axis square
    set(gca,'FontSize',15)
    set(gcf, 'Color','white')
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    zlabel('$l$','interpreter','latex');
    title('Original cost function $l(x)$','interpreter','latex');
end

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


% set up what we want to visualize while computing this
if visualize
    % want to view the set (not function)
    extraArgs.visualize.valueSet = 1;
    extraArgs.visualize.targetSet = 0;

    % view the set at V = 4 (this is the set that we must start in to avoid
    % going more than sqrt(4) meters away from the planner.  The smaller 
    % this number, the more likely the set will "disappear")
    extraArgs.visualize.sliceLevel = 4;
    
    % deletes previous plot at each time step
    extraArgs.visualize.deleteLastPlot = 1;
    
    % tells you how close you are to convergence
    extraArgs.visualize.convergeTitle = 1;
    
    extraArgs.visualize.viewAxis = ...
        [gMin(1) gMax(1) gMin(2) gMax(2) gMin(3) gMax(3)];
    extraArgs.visualize.figNum = 2;
    
    extraArgs.visualize.xTitle = '$r_x$';
    extraArgs.visualize.yTitle = '$r_y$';
    extraArgs.visualize.zTitle = '$\theta$';
    extraArgs.visualize.fontSize = 15;
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

%% Run analysis
tic

% in FaSTrack we want to take the max with the cost function l(x) over time
compMethod = 'maxVWithL';

% The main function to run reachability code
[data, tau] = HJIPDE_solve(data0, tau, sD, compMethod, extraArgs);

runtime = toc;

% Get TEB
small = .1;
TEB = min(sqrt(data(:)))+small;

if visualize
    
    % Plot the converged value function
    figure(1)
    hold on
    subplot(1,2,2)
    [g2D, data2D] = proj(sD.grid, data, [0 0 1], 'min');
    

    % visualize sqrt of value function
    hF = visFuncIm(g2D, sqrt(data2D),color,alpha);
    
    % make pretty
    view(30,10)
    c = camlight;
    c.Position = [4 -2 1];
    axis([gMin(1) gMax(1) gMin(2) gMax(2) 0 3])
    axis square
    set(gca,'FontSize',15)
    set(gcf, 'Color','white')
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    zlabel('$V$','interpreter','latex');
    title('Converged Value function $V(x)$','interpreter','latex');
    
    
    % Plot some levels of the value function
    figure(3)
    clf
    alpha = .2;
    levels = [TEB+.2, TEB+.5, TEB+1];
    
    [g2D, data2D] = proj(sD.grid,data,[0 0 1],0);%'max');
    small = .05;
    subplot(2,3,1)
    h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(1)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, sqrt(data), 'red', levels(1));
    axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small -pi pi])
    axis square
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    zlabel('$l$','interpreter','latex');
    
    subplot(2,3,4)
    h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(1)+small);
    hold on
    h = visSetIm(g2D, sqrt(data2D), 'red', levels(1));
    axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small])
    title(['R = ' num2str(levels(1))])
    axis square
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    
    subplot(2,3,2)
    h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(2)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, sqrt(data), 'red', levels(2));
    axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small -pi pi])
    title(['t = ' num2str(tau(end)) ' s'])
    axis square
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    zlabel('$l$','interpreter','latex');
    
    subplot(2,3,5)
    h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(2)+small);
    hold on
    h = visSetIm(g2D, sqrt(data2D), 'red', levels(2));
    axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small])
    title(['R = ' num2str(levels(2))])
    axis square
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    
    subplot(2,3,3)
    h0 = visSetIm(sD.grid, sqrt(data0), 'blue', levels(3)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(sD.grid, sqrt(data), 'red', levels(3));
    axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small -pi pi])
    axis square
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    zlabel('$l$','interpreter','latex');
    
    subplot(2,3,6)
    h0 = visSetIm(g2D, sqrt(data02D), 'blue', levels(3)+small);
    hold on
    h = visSetIm(g2D, sqrt(data2D), 'red', levels(3));
    axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small])
    title(['R = ' num2str(levels(3))])
    axis square
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    
    set(gcf,'Color','white')
end

end

