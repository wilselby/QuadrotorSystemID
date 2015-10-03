function testlrls2d(centers, pointcounts, variances, lambdas)
% Test harness for 2D RLS (including multiclass).
% testlrls2d(centers, pointcounts, [variances={I,...,I}, lambdas=1])
  
% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

nc = size(centers,1);

% Default arguments.
if (nargin < 2) % pointcounts
  pointcounts = repmat(100,nc,1);
end

if (nargin < 3) % variances
  variances = cell(nc,1);
  I2 = eye(2);
  for i = 1:nc
    variances{i} = I2;
  end
end
variances

if (nargin < 4) % lambdas
  lambdas = [1];
end

nl = length(lambdas);

[X,Y] = gaussianclouds(centers,variances,pointcounts);

plotclouds(X, Y);

[ws,loos] = lrlsloo(X, Y, lambdas);

minx = min(X(:,1));
maxx = max(X(:,1));

for cc = 1:nc
  fmt = sprintf('%s', colorchar(cc));
  for ll = 1:nl
    w = squeeze(ws(ll,cc,:));
    plotline(w(1), w(2), minx, maxx, w(3), fmt);
    hold on;
  end
end

hold off;
