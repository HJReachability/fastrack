%
% Copyright (c) 2018, The Regents of the University of California (Regents).
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
%
%    1. Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%
%    2. Redistributions in binary form must reproduce the above
%       copyright notice, this list of conditions and the following
%       disclaimer in the documentation and/or other materials provided
%       with the distribution.
%
%    3. Neither the name of the copyright holder nor the names of its
%       contributors may be used to endorse or promote products derived
%       from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
%
% Please contact the author(s) of this library if you have any questions.
% Authors: Jaime F. Fisac   ( jfisac@eecs.berkeley.edu )
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Computes the Tracking Error Bound (TEB) for Mass4DRelDubins.
%% The current version of the code is set up as a script, and parameters are
%% defined below——modify the parameters to compute the TEB for different cases.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%function ComputeTEB()
% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'zero' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'zero' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;

% Do not compute optimal trajectory from an initial state
compTraj = false;

%% Grid
grid_min = [ 0.05; -pi; -2; -2];    % Lower corner of computation domain
grid_max = [ 1;  pi;  2;  2];    % Upper corner of computation domain
N = [20; 31; 21; 21];            % Number of grid points per dimension
pdDims = 2;                      % 2nd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims); % create state space grid


%% time vector
t0 = 0;
tMax = 0.5;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% Tracker input bound
a_max_ = 2; % ~10º --> ~0.2 rad --> ~0.2 g --> ~2 m/s^2

% Dubins input bound
omega_max = pi/4; % rad/s

% Disturbance bound
d_max = 0.5; % m/s^2

% Dubins car speed
v = 0.5; % m/s

% surface function is the distance between the point mass and the Dubins car
data0 = g.xs{1};

% Roles of the control and disturbance for the (original) surface function
uMode = 'min';
dMode = 'max';

%% --- Begin negated computation ---
% Since we are computing the *maximum* future payoff, but the LST only natively
% supports the lower obstacle saturation, we need to negate the target surface
% function.
data0 = -data0;

% The control and disturbance roles are switched: u maximizes *negated* distance
uMode = 'max';
dMode = 'min';

% Define dynamic system
mass_vs_dubins = Mass4DRelDubins([0, 0, 0, 0], a_max_, omega_max, d_max, v);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = mass_vs_dubins;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;

% Define type of Reachable Set to compute.
minWith = 'zero'; % Compute reachable tube (for all time, not only final time)

%% Compute value function.

HJIextraArgs.visualize = true; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update

HJIextraArgs.plotData.plotDims = [1 1 0 0];
HJIextraArgs.plotData.projpt = 'min';
HJIextraArgs.viewAngle = [0,90];

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);

% Un-negate value function to restore normal conditions.
data0 = -data0;
data  = -data;
uMode = 'min';
dMode = 'max';
% --- End negated computation ---

%% Plot surface function l(x) and value function V(x).

[h_f, h_data, h_data0] = PlotValueXY(g,data,data0);

%% Save data to load into FaSTrack.

% Keep final step of value function computation only
data = squeeze(data(:,:,:,:,end));

% Choose TEB level set relative to the minimum nonempty level set
level_set_margin = 0.1;

% Define transition between free and forced control near the boundary
threshold_lower = 0.1;
threshold_upper = 0.8;
priority_lower = min(data(:)) + level_set_margin*threshold_lower;
priority_upper = min(data(:)) + level_set_margin*threshold_upper;

% Grid size and bounds
num_cells = N;
lower = grid_min;
upper = grid_max;

%% TODO: define tracker_params, planner_params, bound_params consistent with
% the Initialize() function in the appropriate class

% Save in .mat file

save value_function priority_upper priority_lower num_cells lower upper data

% end