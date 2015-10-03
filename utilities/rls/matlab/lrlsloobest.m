function [ws,loos,bestlambdas] = lrlsloobest(X, Y, lambdas)
% Wrapper around lrlsloo that gives only the w and LOO for the
% "best" lambda (lowest LOO) for each rhs.  Also returns the
% best lambda.
%
% See lrlsloo for more in-depth explanations of parameter settings.
%
% If X is n by d and Y is n by cl, then ws is d by cl and loos is 
% cl by 1.

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

if (nargin < 3)
  lambdas = logspace(-6,6,30);
end


[wsall,loosall] = lrlsloo(X, Y, lambdas);

% Variables checked in lrlsloo above.
d = size(X,2);
cl = size(Y,2);

ws = zeros(d,cl);
loos = zeros(cl,1);
bestlambdas = zeros(cl,1);

for cc = 1:cl
  [minval,minind] = min(loosall(:,cc));
  ws(:,cc) = wsall(minind,cc,:);
  loos(cc) = minval;
  bestlambdas(cc) = lambdas(minind);
end
