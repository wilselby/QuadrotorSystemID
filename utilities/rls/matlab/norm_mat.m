function [N,m,v] = norm_mat(M,m,v)
% Normalizes a matrix so that each COLUMN has mean 0 and norm 1.
% Takes and returns means and variances, so can be applied 
% to test sets.

[pts dims] = size(M);
opts = ones(pts,1);

if (nargin < 3)
  m = mean(M);
  v = var(M);
end

N = (M - opts*m)./(opts * sqrt(v));
