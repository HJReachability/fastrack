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
%% Plots the value function from the Tracking Error Bound (TEB) computation
%% for Mass4DRelDubins, as well as optionally the payoff surface function.
%% The plot is shown in XY coordinates with the planner assumed to be facing
%% in the positive X direction. The polar grid is appropriately transformed.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [h_f, h_data, h_data0, C, h_c] = PlotValueXY(g,data,data0,mode,high_dim_vals)
%% Plot surface function l(x) and value function V(x).
%  NOTE: As a sanity check, it should always be V(x) >= l(x).

if nargin >=3 && ~isempty(data0)
	plot_data0 = true;
else
	plot_data0 = false;
end

if nargin < 4
	mode = 'min';
end

if nargin < 5
    % TODO: @Jaime make vals the actual argument, not idx
	high_dim_vals = [ceil(g.N(3)/2), ceil(g.N(4)/2)];
end

% Transform from polar coordinates
Xs = g.xs{1}.*cos(g.xs{2});
Ys = g.xs{1}.*sin(g.xs{2});

% Re-append first "column" at the end to close the periodic dimension
Xs_closed = [Xs, Xs(:,1,:,:)];
Ys_closed = [Ys, Ys(:,1,:,:)];
if plot_data0
	data0_closed = [data0, data0(:,1,:,:)];
end
data_closed  = [data, data(:,1,:,:,:)];

% Determine how to treat the higher dimensions
if strcmp(mode,'min') % projection of higher dimensions (minimum)
	data0_low_dim = min( min(data0_closed,[],4), [],3);
	data_low_dim  = min( min(data_closed,[],4), [],3);
elseif strcmp(mode,'slice') % section of higher dimensions
	data0_low_dim = data0_closed(:,:,high_dim_vals(1),high_dim_vals(2));
	data_low_dim  = data_closed(:,:,high_dim_vals(1),high_dim_vals(2));
else
	error('mode %s undefined.',mode);
end

% Initialize figure and handles
h_f = figure;
hold on
h_data  = 0;
h_data0 = 0;

% Plot payoff surface function (signed distance) if provided
if plot_data0
	h0 = mesh(Ys_closed(:,:,1,1),...
	          Xs_closed(:,:,1,1),...
	          data0_low_dim,...
	          'FaceColor','none');
end

% Plot value function
h  = surf(Ys_closed(:,:,1,1),...
          Xs_closed(:,:,1,1),...
          data_low_dim);

[C,h_c]  = contour3(Ys_closed(:,:,1,1),...
          Xs_closed(:,:,1,1),...
          data_low_dim,...
          min(data_low_dim(:)):0.01:max(data_low_dim(:)));

% Adjust axes and perspective
h_a = h_f.CurrentAxes;
h_a.View = [75 20];