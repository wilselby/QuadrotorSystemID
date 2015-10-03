function [ws,loos] = lrlsloo_ll(X, U, S2, Y, lambdas)
% A "lower level" function that does the work in lrlsloo.
% Assumes you've already performed an SVD of X = USV^t, and
% that S2 is a row vector containing the squared singular
% values.
%
% Does no error checking on arugment sizes, all arguments
% must be specified.  See lrlsloo.m for more info.

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

[n,d] = size(X);
cl = size(Y,2);
l = length(lambdas);

% Result allocation
ws = zeros(l,cl,d);
loos = zeros(l,cl);
loos(:,:) = Inf;

for ll = 1:l
  [wsll,looerrsll] = lrlsloo_ll1(X,U,S2,Y,lambdas(ll));
  ws(ll,:,:) = wsll;
  loos(ll,:) = sqrt(sum(looerrsll.^2,1));
end
