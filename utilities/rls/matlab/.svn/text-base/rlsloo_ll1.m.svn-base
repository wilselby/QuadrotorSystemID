function [cs,looerrs] = rlsloo_ll1(V, D, Y, lambda)
% Computes cs and the actual LOO errors for a single value of lambda.
% K is n by n, D is 1 by n, Y is n by cl, and lambda is a scalar.
%  
% cs is cl by n, and looerrs is n by cl.
  
% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

n = size(V,1);
cl = size(Y,2);

VtY = V'*Y;
inner = 1./(D + lambda);
ViD = V.*(ones(n,1)*inner);

cs = ViD*VtY;
dGi = sum(ViD.*V,2);

looerrs = cs./(dGi*ones(1,cl));
cs = cs';  % For later storage consistency reasons.
