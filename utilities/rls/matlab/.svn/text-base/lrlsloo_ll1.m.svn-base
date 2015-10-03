function [ws,looerrs] = lrlsloo_ll1(X, U, S2, Y, lambda)
% Computes ws and the actual LOO errors for a single value of lambda.
% X and U are n by d, S is a row-vector of length d, Y is n by cl,
% and lambda is a scalar.
%  
% ws is cl by d, and looers is n by cl.
  
% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

n = size(X,1);
cl = size(Y,2);

UtY = U'*Y;

inner = (1./(S2 + lambda) - 1/lambda);
Uinner = U.*(ones(n,1)*inner);

c = Y/lambda + Uinner*UtY;
dGi = 1/lambda + sum(Uinner.*U,2);

looerrs = c./(dGi*ones(1,cl));
ws = c'*X;
