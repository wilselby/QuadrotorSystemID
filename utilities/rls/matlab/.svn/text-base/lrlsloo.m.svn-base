function [ws,loos] = lrlsloo(X, Y, lambdas)
% Linear regularized least squares.
%
% X is a data matrix whose ROWS are the data points.
% If we have n points in d dimensions, X is n by d.
% Y are the "labels".   Y is n by cl, where cl is the number
% of "classes" (the number of regression problems to be
% solved).
%
% lambdas is a vector of length l, containing the different
% regularization parameters to try.  DEFAULT: logspace(-6,6,30).
%
% Results:
%
% ws is a matrix of size (l,c,d), where w(i,j,:) is a d-vector which
% is the linear function learned by linear RLS with lambda = lambdas(i) 
% and y = Y(:,j).
%
% loos is a matrix of size (l,c), where loos(i,j) is the norm of 
% the total LOO error vector for
% linear RLS with lambda = lambdas(i) and y = Y(:,j).

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

% Argument handling
if (nargin < 2)
  error('X and Y are required arguments to lrlsloo.');
end

if (nargin < 3)
  lambdas = logspace(-6,6,30);
end

% Variable checking
[n,d] = size(X);
if (size(Y,1) ~= n)
  error('lrlsloo: X and Y must have the same number of rows.');
end

if (length(lambdas) ~= prod(size(lambdas)))
  error('lrlsloo: lambdas must be a vector.');
end
l = length(lambdas);

[U,S2] = lrls_us2(X);
[ws,loos] = lrlsloo_ll(X,U,S2,Y,lambdas);
