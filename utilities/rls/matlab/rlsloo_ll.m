function [cs,loos] = rlsloo_ll(V, D, Y, lambdas)
% A "lower level" function that does the work in rlsloo.
% Assumes you've already performed an eignedecomposition
% of K = VDV^t, and that D is a row vector containing the eigenvalues.
%
% Does no error checking on arugment sizes, all arguments
% must be specified.  See rlsloo.m for more info.

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

n = size(V,1);
cl = size(Y,2);
l = length(lambdas);

% Result allocation
cs = zeros(l,cl,n);
loos = zeros(l,cl);
loos(:,:) = Inf;

for ll = 1:l
  [csll, looerrsll] = rlsloo_ll1(V,D,Y,lambdas(ll));
  cs(ll,:,:) = csll;
  loos(ll,:) = sqrt(sum(looerrsll.^2,1));
end
