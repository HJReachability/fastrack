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
%% Computes the optimal point mass control input for Mass4DRelDubins.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function uOpt = optCtrl(obj, ~, x, deriv, uMode, ~)
% uOpt = optCtrl(obj, t, x, deriv, uMode, dMode, MIEdims)

% Note: dynamics specified in dynamics.m

%% Input processing
if nargin < 5
  uMode = 'min';
end

%% Optimal control
if iscell(deriv)
  uOpt = cell(obj.nu, 1);
  normalizer = max(sqrt(deriv{3}.^2 + deriv{4}.^2), 1e-6);
  if strcmp(uMode, 'max')
    uOpt{1} = deriv{3} ./ normalizer * obj.a_max_;
    uOpt{2} = deriv{4} ./ normalizer * obj.a_max_;

  elseif strcmp(uMode, 'min')
    uOpt{1} = -deriv{3} ./ normalizer * obj.a_max_;
    uOpt{2} = -deriv{4} ./ normalizer * obj.a_max_;
  else
    error('Unknown uMode!')
  end

else
  uOpt = zeros(obj.nu, 1);
  normalizer = max(sqrt(deriv(3)^2 + deriv(4)^2), 1e-6);
  if strcmp(uMode, 'max')
    uOpt(1) = deriv(3) / normalizer * obj.a_max_;
    uOpt(2) = deriv(4) / normalizer * obj.a_max_;

  elseif strcmp(uMode, 'min')
    uOpt(1) = -deriv(3) / normalizer * obj.a_max_;
    uOpt(2) = -deriv(4) / normalizer * obj.a_max_;
  else
    error('Unknown uMode!')
  end
end




end
