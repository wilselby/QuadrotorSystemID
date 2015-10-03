function [cs,loos] = rlsloo(K, Y, lambdas)
% Nonlinear regularized least squares.
%
%
% K is an n by n symmetric kernel matrix.  Y is n by cl, where cl is the
% number of "classes" (the number of regression problems to be solved.)
%
% lambdas is a vector of length l, containing the different
% regularization parameters to try.  DEFAULT: logspace(-6,6,30).
%
% Results:
%
% cs is a matrix of size (l,c,n), where cs(i,j,:) is a n-vector
% representing the function weights for rls with lambda = lambdas(i) and
% y = Y(:,j):
%       f_{ij}(x) = \sum_p cs(i,j,p) k(x_p, x)
%
% loos is a matrix of size (l,c), where loos(i,j) is the norm of 
% the total LOO error vector for nonlinear RLS with lambda = lambdas(i) 
% and y = Y(:,j).

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

% Argument handling
if (nargin < 2)
  error('K and Y are required arguments to rlsloo.');
end

if (nargin < 3)
  lambdas = logspace(-6,6,30);
end

% Variable checking
[n,n2] = size(K);
if (n ~= n2)
  error('rlsloo: K must be square.');
end

if (size(Y,1) ~= n)
  error('rlsloo: K and Y must have the same number of rows.');
end

if (length(lambdas) ~= prod(size(lambdas)))
  error('lrlsloo: lambdas must be a vector.');
end
l = length(lambdas);

[V,D] = rls_vd(K);
[cs,loos] = rlsloo_ll(V,D,Y,lambdas);
