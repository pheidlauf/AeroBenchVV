function [x,options,P1,P2,yinit,ylast] = fminsa(funfcn,x,options,grad,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10)
%FMINSA	Minimize a function of several variables.
%	FMINSA('F',X0) attempts to return a vector x which is a local minimizer 
%	of F(x) near the starting vector X0.  'F' is a string containing the
%	name of the objective function to be minimized.  F(x) should be a
%	scalar valued function of a vector variable.
%
%	FMINSA('F',X0,OPTIONS) uses a vector of control parameters.
%	If OPTIONS(1) is nonzero, intermediate steps in the solution are
%	displayed; the default is OPTIONS(1) = 0.  OPTIONS(2) is the termination
%	tolerance for x; the default is 1.e-4.  OPTIONS(3) is the termination
%	tolerance for F(x); the default is 1.e-4.  OPTIONS(14) is the maximum
%	number of steps; the default is OPTIONS(14) = 500.  The other components
%	of OPTIONS are not used as input control parameters by FMIN.  For more
%	information, see FOPTIONS.
%
%	FMINSA('F',X0,OPTIONS,[],P1,P2,...) provides for up to 10 additional
%	arguments which are passed to the objective function, F(X,P1,P2,...)
%
%       FMINSA is a modified form of the MATLAB FMINS. Modified on 27 Jan 97 by 
%       Chris Shearer, New outputs were added as follows
%       P1 = State vector(x) in dx = Ax + Bu
%       P2 = Control vector(u)
%       yinit = inital value of cost function
%       ylast = last value of cost function
%
%	FMINS uses a Simplex search method.
%
%	See also FMIN. 

%	Reference: J. E. Dennis, Jr. and D. J. Woods, New Computing
%	Environments: Microcomputers in Large-Scale Computing,
%	edited by A. Wouk, SIAM, 1987, pp. 116-122.

%	C. Moler, 8-19-86
%	Revised Andy Grace, 6-22-90, 1-17-92 CBM,  10-5-93 AFP
%	Copyright (c) 1984-94 by The MathWorks, Inc.

if nargin<3, options = []; end
options = foptions(options);
prnt = options(1);
tol = options(2);
tol2 = options(3);
% The input argument grad is there for compatability with FMINU in
% the Optimization Toolbox, but is not used by this function.

evalstr = [funfcn];
if ~any(funfcn<48)
    evalstr=[evalstr, '(x'];
    for i=1:nargin - 4
        evalstr = [evalstr,',P',int2str(i)];
    end
    evalstr = [evalstr, ')'];
end

n = prod(size(x));
if (~options(14)) 
    options(14) = 200*n; 
end

% Set up a simplex near the initial guess.
xin = x(:); % Force xin to be a column vector
v = xin;    % Place input guess in the simplex! (credit L.Pfeffer at Stanford)
x(:) = v; [fv,P1,P2] = eval(evalstr); 

yinit = fv;  % Save the inital cost function value, cms

% Following improvement suggested by L.Pfeffer at Stanford
usual_delta = 0.05;             % 5 percent deltas for non-zero terms
zero_term_delta = 0.00025;      % Even smaller delta for zero elements of x
for j = 1:n
   y = xin;
   if y(j) ~= 0
      y(j) = (1 + usual_delta)*y(j);
   else
      y(j) = zero_term_delta;
   end
   v = [v y];
   x(:) = y; [f,P1,P2] = eval(evalstr);
   ylast = f;
   fv = [fv  f];
end
[fv,j] = sort(fv);
v = v(:,j);


cnt = n+1;
if prnt
   clc
   format compact
   format short e
   home
   cnt
   disp('initial ')
   disp(' ')
   v
   f
end

alpha = 1;  beta = 1/2;  gamma = 2;
[n,np1] = size(v);
onesn = ones(1,n); 
ot = 2:n+1;
on = 1:n;

% Iterate until the diameter of the simplex is less than tol.
while cnt < options(14)
    if max(max(abs(v(:,ot)-v(:,onesn)))) <= tol & ...
           max(abs(fv(1)-fv(ot))) <= tol2
        break
    end

    % One step of the Nelder-Mead simplex algorithm

    vbar = (sum(v(:,on)')/n)';
    vr = (1 + alpha)*vbar - alpha*v(:,n+1);
    x(:) = vr;
    [fr,P1,P2] = eval(evalstr); 
    ylast = fr;
    cnt = cnt + 1; 
    vk = vr;  fk = fr; how = 'reflect ';
    if fr < fv(n)
        if fr < fv(1)
            ve = gamma*vr + (1-gamma)*vbar;
            x(:) = ve;
            [fe,P1,P2] = eval(evalstr);
            ylast = fe;
            cnt = cnt + 1;
            if fe < fv(1)
                vk = ve; fk = fe;
                how = 'expand  ';
            end
        end
    else
        vt = v(:,n+1); ft = fv(n+1);
        if fr < ft
            vt = vr; ft = fr;
        end
        vc = beta*vt + (1-beta)*vbar;
        x(:) = vc;
        [fc,P1,P2] = eval(evalstr); 
        ylast = fc;
        cnt = cnt + 1;
        if fc < fv(n)
            vk = vc; fk = fc;
            how = 'contract';
        else
            for j = 2:n
                v(:,j) = (v(:,1) + v(:,j))/2;
                x(:) = v(:,j);
                [fv(j),P1,P2] = eval(evalstr); 
                ylast = fv(j);
            end
        cnt = cnt + n-1;
        vk = (v(:,1) + v(:,n+1))/2;
        x(:) = vk;
        [fk,P1,P2] = eval(evalstr); 
        ylast = fk;
        cnt = cnt + 1;
        how = 'shrink  ';
        end
    end
    v(:,n+1) = vk;
    fv(n+1) = fk;
    [fv,j] = sort(fv);
    v = v(:,j);

    if prnt
        home
        cnt
        disp(how)
        disp(' ')
        v
        fv
    end
end
x(:) = v(:,1);
if prnt, format, end
options(10)=cnt;
options(8)=min(fv); 
if cnt==options(14) 
    if options(1) >= 0
        % Commented out to keep script running quickly
%         disp(['Warning: Maximum number of iterations (', ...
%                int2str(options(14)),') has been exceeded']);
%         disp( '         (increase OPTIONS(14)).')
    end
end



