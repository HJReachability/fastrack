%
% Copyright (c) 2017, The Regents of the University of California (Regents).
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
% Authors: Jaime Fernandez Fisac   ( jfisac@eecs.berkeley.edu )
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% LQR state feedback matrix computation for a 7-state near-hover quadrotor
% around the zero-yaw angle.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function K = crazyflie_lqr_compute(Q,R,filename,col_separator,row_separator)
	% brief: Computes state feedback matrix for near-hover quadrotor dynamics
	% input:
	%			  Q:	matrix (or diagonal) of quadratic state cost weights
	%			  R:	matrix (or diagonal) of quadratic control cost weights
	%	   filename:	name of text file to write output to
	% col_separator:	string to use to separate columns in generated file
	% row_separator:	string to use to separate rows in generated file

	%% System dynamics
	% (quadrotor in near hover, assuming small attitude angles)
	%    dx/dt     =    vx_G
	%    dy/dt     =    vy_G 	<-- _G stands for global frame (vs body)
	%    dz/dt     =    vz_G
	%    dvx_G/dt  =    T*sin(theta)*cos(psi) + T*sin(phi)*sin(psi)      ~=    g * theta
	%    dvy_G/dt  =  - T*sin(phi)*cos(psi)   + T*sin(theta)*sin(psi)	 ~=  - g * phi
	%    dvz_G/dt  =    T*cos(phi)*cos(theta) - g                        ~=    T - g
	%    dpsi/dt   =    w 
	%
	%    control: u = [ phi, theta, w, T]
	%
	% Linearized dynamics (around hover equilibrium point, NOT zero input)
	%
	%	dx/dt = Ax + Bu
	%
	% with x = [x, y, z, vx_b, vy_b, vz_b, psi], u = [phi, theta, w, T - g].
	% Order of controls as per crazyflie_server: [roll, pitch, yawrate, thrust].
	%
	% Note 1: angle sign convention is based on positive (right-hand) angles on FLU frame
	% (phi ROLL RIGHT / theta PITCH DOWN / psi YAW left)
	%
    % Note 2: the total (nonlinear) thrust command must be constructed as T = u(4) + g.

	g = 9.81;

	%		x	y 	z 		vx 	vy  vz 		psi
	A = [ 	0	0	0		1	0	0		0;		% x
			0	0	0		0	1	0		0;		% y
			0	0	0		0	0	1		0;		% z
			0	0	0		0	0	0		0;		% vx_b
			0	0	0		0	0	0		0;		% vy_b
			0	0	0		0	0	0		0;		% vz_b
			0	0	0		0	0	0		0];		% psi

	%		ph  th  w   T-g
	B = [	0	0	0	0;		% x
			0	0	0	0;		% y
			0	0	0	0;		% z
			0   g 	0	0;		% vx_b
           -g 	0	0	0;		% vy_b
			0 	0 	0	1;		% vz_b
			0	0	1	0];		% psi

	%% Objective function
	%
	%	  /inf
	% J = |		( x(t)' * Q * x(t)  +  u(t)' * R * u(t) ) dt
	%     /0

    if nargin < 1
	    %			x	y 	z 		vx 	vy  vz 		psi
	    Q = diag([	1	1	2		1	1	2		10	].^2);
    elseif min(size(Q))<2
        Q = diag(Q);
    end

    if nargin < 2
	    %			ph  	th  	w   	T
	    R = diag([	0.1 	0.1 	0.1 	0.1	].^2);
	elseif min(size(R))<2
        R = diag(R);
    end

	% Solve Continuous-time Algebraic Riccati Equation (CARE)
	% (MATLAB uses the Schur method on the Hamiltonian matrix)
	[~,~,K] = care(A,B,Q,R);    % SIGN: this function uses the convention u = -K*x
    K = -K;                     % SIGN: switch to convention u = K*x by flipping sign of K
    
    if nargin >= 3
    	if nargin < 4
    		col_separator = '	';
    	end
    	if nargin < 5
    		row_separator = '\n';
    	end
        fileID = fopen(filename,'w');
        for i=1:size(K,1)
	        for j=1:size(K,2)
				fprintf(fileID,'%6.2f',K(i,j));
				if j < size(K,2)
					fprintf(fileID,col_separator);
				elseif i < size(K,1)
					fprintf(fileID,row_separator);
				end
			end
		end
		fclose(fileID);
    end
end