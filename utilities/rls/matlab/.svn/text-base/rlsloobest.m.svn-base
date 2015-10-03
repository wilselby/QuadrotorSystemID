function [cs,loos,bestlambdas] = rlsloobest(K, Y, lambdas)
% function [cs,loos,bestlambdas] = rlsloobest(K, Y, lambdas)
% Wrapper around rlsloo that gives only the c and LOO for the
% "best" lambda (lowest LOO) for each rhs.  Also returns the
% best lambda.
%
% See lrlsloo for more in-depth explanations of parameter settings.
%
% If K is n by n and Y is n by cl, then cs is n by cl and loos is 
% cl by 1.

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).
  
if (nargin < 3)
  lambdas = logspace(-6,6,30);
end

[csall,loosall] = rlsloo(K, Y, lambdas);

% Variables checked in lrlsloo above.
n = size(K,1);
cl = size(Y,2);

cs = zeros(n,cl);
loos = zeros(cl,1);
bestlambdas = zeros(cl,1);

for cc = 1:cl
  [minval,minind] = min(loosall(:,cc));
  cs(:,cc) = csall(minind,cc,:);
  loos(cc) = minval;
  bestlambdas(cc) = lambdas(minind);
end
