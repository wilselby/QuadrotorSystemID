function looerrs = loodirect(K, Y, lambda)
% loodirect(K, Y, lambda)
% Computes loo values "directly" (by removing the i'th point,
% training on the remaining n-1 points, and computing the output
% value at the i'th point.  Is returning LOO errors (LOOVALS - y).
% Slow, but useful for testing.
%
% K is n by n, Y is n by cl, and lambda is scalar.

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

[n,cl] = size(Y);

loopreds = zeros(n,cl);

In1 = eye(n-1);
for pt = 1:n
  inds = [1:(pt-1) (pt+1:n)];
  Ksub = K(inds,inds) + In1;
  Kprods = K(inds,pt);
  loopreds(pt, :) = (Ksub\Y(inds,:))'*Kprods;
end

looerrs = Y - loopreds;
