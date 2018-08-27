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
%% Defines the Mass4DRelDubins MATLAB class, which inherits from the DynSys
%% base class. This class assumes Dubins car planner dynamics and planar
%% point mass (double integrator) tracker dynamics and uses a Circle tracking
%% error bound. Mass4D differs from Quad4D only in that its acceleration vector
%% is constrained to lie in a circle instead of a box.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef Mass4DRelDubins < DynSys
% Mass4DRelDubins: Dynamics of a planar point mass pursuing a Dubins car.

  properties
    %% Dubins car (fixed) velocity
    v_

    %% Mass4D maximum acceleration control (Euclidean norm)
    a_max_

    % Dubins maximum angular rate (absolute value)
    omega_max_

    % Mass4D maximum acceleration disturbance (Euclidean norm)
    d_max_

  end

  methods
    function obj = Mass4DRelDubins(x, a_max, omega_max, d_max, v)
      % obj = Mass4DRelDubins(x, Mass4D, KinVeh2D)
      %
      % Constructor. Creates the dynamical system object with state x and
      % parameters from the input parameters
      %
      % Inputs:
      %   x           - state: [distance, bearing, tangent_vel, normal_vel]
      %   a_max       - maximum Mass4D acceleration control
      %   omaga_max   - maximum Dubins angular rate
      %   d_max       - maximum Mass4D acceleration disturbance
      %
      % Output:
      %   obj         - Mass4DRelDubins object
      %
      % Note: dynamics specified in dynamics.m

      if numel(x) ~= 4
        error('Initial state does not have right dimension!');
      end

      if ~iscolumn(x)
        x = x';
      end

      obj.x = x;
      obj.xhist = obj.x;

      obj.a_max_ = a_max;
      obj.omega_max_ = omega_max;
      obj.d_max_ = d_max;
      obj.v_ = v;

      obj.pdim = [1,3,4];
      obj.hdim = 2;

      obj.nx = 4;
      obj.nu = 2;
      obj.nd = 3;
    end

  end % end methods
end % end classdef
