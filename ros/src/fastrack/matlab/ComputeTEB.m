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

%% Problem parameters.

% Gravity
gravity = 9.81; % m/s^2


% ---- Planner parameters ----

% Dubins turning rate bound.
omega_max_ = pi; % rad/s

% Dubins car speed.
v_ = 1.0; % m/s

% Vertical planner speed (for analytic vertical component of TEB).
v_z_ = 0.5; % m/s


% ---- Tracker parameters ----

% Horizontal acceleration control bounds (via pitch/roll).
pitch_roll_radius_ = 0.2; % rad;  ~10º --> ~0.2 rad --> ~0.2 g --> ~2 m/s^2
a_max_ = gravity*tan(pitch_roll_radius_); % m/s^2; g*tan(0.2) = 1.99 m/s^2

% Vertical and directional parameters (for analytic vertical component of TEB).
yaw_rate_max_ = pi;  % rad/s
thrust_min_ = gravity - 3; % m/s^2
thrust_max_ = gravity + 6; % m/s^2

% Horizontal disturbance bound (maximum radius of horizontal component).
d_max_ = 0.5; % m/s^2

% Vertical disturbance bound (symmetrical interval).
d_z_max_ = 0.5; % m/s^2


% ---- Differential game setup ----

% State grid.
grid_min = [ 0.01; -pi; -1; -1];  % Lower corner of computation domain.
grid_max = [ 0.5;  pi;  1;  1];   % Upper corner of computation domain.
N = [50; 35; 21; 21];             % Number of grid points per dimension.
pdDims = 2;                       % 2nd dimension is periodic.
g = createGrid(grid_min, grid_max, N, pdDims); % Create state space grid.

% Time vector.
t0 = 0;
tMax = 0.5;
dt = 0.05;
tau = t0:dt:tMax;

% Payoff surface function: distance between the point mass and the Dubins car.
data0 = g.xs{1};

% Roles of the control and disturbance for the (non-negated) surface function.
uMode = 'min';
dMode = 'max';

% HelperOC config. Do not compute optimal trajectory from an initial state.
compTraj = false;


%% Hamilton-Jacobi Computation of Tracking Error Bound (TEB)

% ------------------------- Begin negated computation -------------------------
% Since we are computing the *maximum* future payoff, but the LST only natively
% supports the lower obstacle saturation, we need to negate the target surface
% function.
data0 = -data0;

% Control and disturbance roles are switched: u maximizes *negated* distance.
uMode = 'max';
dMode = 'min';

% Define dynamic system.
mass_vs_dubins = Mass4DRelDubins([0, 0, 0, 0], a_max_, omega_max_, d_max_, v_);

% Put grid and dynamic systems into schemeData.
schemeData.grid = g;
schemeData.dynSys = mass_vs_dubins;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;

% Define type of Reachable Set to compute.
minWith = 'zero'; % Compute reachable tube (for all time, not only final time)

% Set up value function computation options.
HJIextraArgs.visualize = true; % Show plot
HJIextraArgs.fig_num = 1; % Set figure number
HJIextraArgs.deleteLastPlot = true; % Delete previous plot as you update

HJIextraArgs.plotData.plotDims = [1 1 0 0];
HJIextraArgs.plotData.projpt = 'min';
HJIextraArgs.viewAngle = [0,90];

% Compute value function.
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);

% Un-negate value function to restore normal conditions.
data0 = -data0;
data  = -data;
uMode = 'min';
dMode = 'max';
% -------------------------- End negated computation --------------------------

% Keep final step of value function computation only.
data = squeeze(data(:,:,:,:,end));


%% Plotting

% Plot surface function l(x) and value function V(x).
[h_f, h_data, h_data0] = PlotValueXY(g,data,data0);


%% Post-process data for saving and loading into FaSTrack.

% Choose TEB level set relative to the minimum nonempty level set.
level_set_margin = 0.1; % in meters
level_set_boundary_value = min(data(:)) + level_set_margin;

% Define transition between free and forced control near the boundary.
threshold_lower = 0.1;
threshold_upper = 0.8;
priority_lower = min(data(:)) + level_set_margin*threshold_lower;
priority_upper = min(data(:)) + level_set_margin*threshold_upper;

% Grid size and bounds.
num_cells = cast(N,'uint64');
lower = grid_min;
upper = grid_max;

%----------------------- Circumcircle TEB approximation -----------------------%
% Compute circle approximation of the TEB in relative position space.
% (This projects the velocity components out, which is done through a min.)
data_project_to_position = min( min(data,[],4), [],3);

% Create array of points in the TEB.
teb_points = [];
for i_rho = 1:N(1)
    for i_theta = 1:N(2)
        if data_project_to_position(i_rho,i_theta) <= level_set_boundary_value
            rho = g.vs{1}(i_rho);
            theta = g.vs{2}(i_theta);
            teb_points = [teb_points; rho, theta];
        end
    end
end

% Compute circumscribed circle of the projected TEB (we know it is symmetrical).
R_c_sq = inf; % Initialize squared circumradius.
x_c = [];     % Initialize circumcenter.
rhos = teb_points(:,1);
thetas = teb_points(:,2);
res_x = 0.01; % 0.01 m resolution
% Iterate over candidate center positions (linear pass).
for x = -grid_max(1):res_x:grid_max(1)
    % Compute radius for this center as distance to farthest point in the set.
    d_sq = max( (rhos.*sin(thetas) ).^2 + (rhos.*cos(thetas) - x).^2 );
    if d_sq < R_c_sq
        R_c_sq = d_sq;
        x_c = x;
    end
end
R_c = sqrt(R_c_sq);

% Semi-height of vertical TEB dimension.
% Based on analytic double-integrator value function.
% The limiting factor is the weakest acceleration capability of both directions.
semi_height_ = v_z_^2 / (min(gravity - thrust_min_,...
                            thrust_max_ - gravity) - d_z_max_); % m

% Control bound parameters for the planner: scalar_bound_interval, [min, max].
planner_params = [v_, -omega_max_, omega_max_];

% Control bound parameters for the tracker: quadrotor_control_bound_cylinder,
% [pitch-roll radius, min yaw rate, min thrust, max yaw rate, max thrust].
tracker_params = [pitch_roll_radius_,...
                  -yaw_rate_max_,...
                  thrust_min_,...
                  yaw_rate_max_,...
                  thrust_max_];

% TEB parameters: tracking_bound_cylinder
% (parametrized TEB approximation: circumscribed circle).
if abs(x_c) > res_x
    warning(...
        ['Circumcircle approximation to TEB is not centered on the origin.\n'...
        'Current implementation assumes that it is. Check value function.'])
end
bound_params   = [R_c, semi_height_];

% Value function (flattened and grid copies)
data_flat = data(:);
data_grid = reshape(data,N(:)');

% Value function gradient (each component flattened)
[deriv, ~, ~] = computeGradients(g, data);
for i = 1:g.dim
  assignin('caller',sprintf('deriv_%i',i-1),deriv{i}(:));
end


%% Save data to .mat file

% Flatten value function
data = data_flat;

file_name = 'value_function'; % Name of file to store data and parameters.
save(file_name, ...
      'priority_upper', ...
      'priority_lower', ...
      'num_cells', ...
      'lower', ...
      'upper', ...
      'data', ...
      'planner_params', ...
      'tracker_params', ...
      'bound_params');
for i = 1:g.dim
  save(file_name,sprintf('deriv_%i',i-1),'-append');
end

% Re-define data as non-flattened in the workspace for other potential uses.
data = data_grid;

% end
