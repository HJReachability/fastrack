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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Defines the dynamics for the Mass4DRelDubins MATLAB class, consisting in
%% the relative dynamics between a planar point mass tracker and a Dubins car
%% planner. The reference frame is aligned with the Dubins car, with velocities
%% and accelerations expressed in the inertial frame (but along the body axes).
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dx = dynamics(obj, ~, x, u, d, ~)
% State
%   x(1): distance (r)
%   x(2): bearing (theta)
%   x(3): tangent_velocity (v_t)
%   x(4): normal_velocity  (v_n)
%
% Tracker control (note this requires outside translation into tracker frame)
%   u(1): tangent_acceleration (a_t)
%   u(2): normal_acceleration (a_n)
%
% Planner control
%   d(1): angular_rate (omega)
%
% Disturbance input
%   d(2): tangent_acceleration_disturbance (d_t)
%   d(3): normal_acceleration_disturbance (d_n)
%
% Parameters
%   v_: Dubins car (fixed) speed
%
% Dynamics
%   r_dot     = (v_t - v_) * cos(theta) + v_n * sin(theta)
%   theta_dot = -omega + ( -(v_t-v_) * sin(theta) + v_n * cos(theta) ) / r
%   v_t_dot   = a_t + d_t + v_n * omega
%   v_n_dot   = a_n + d_n - v_t * omega
%
% Input constraints (for reference only)
%   omega \in Ball(0, omega_max_)
%   (a_t, a_n) \in Ball(0,a_max_)
%   (d_t, d_n) \in Ball(0,d_max_)
%         


if numel(u) ~= obj.nu
  error('Incorrect number of control dimensions!')
end


if iscell(x)
  dx = cell(obj.nx, 1);

  dx{1}   = (x{3}-obj.v_) .* cos(x{2}) + x{4} .* sin(x{2});
  dx{2}   = -d{1} + ( -(x{3}-obj.v_) .* sin(x{2}) + x{4} .* cos(x{2}) ) ./ x{1};
  dx{3}   = u{1} + d{2} + x{4} .* d{1};
  dx{4}   = u{2} + d{3} - x{3} .* d{1};

else
  dx = zeros(obj.nx, 1);

  dx(1)   = (x(3)-obj.v_) * cos(x(2)) + x(4) .* sin(x(2));
  dx(2)   = -d(1) + ( -(x(3)-obj.v_) * sin(x(2)) + x(4) * cos(x(2)) ) / x(1);
  dx(3)   = u(1) + d(2) + x(4) * d(1);
  dx(4)   = u(2) + d(3) - x(3) * d(1);
end


end