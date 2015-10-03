function [X,Y] = gaussianclouds(centers, variances, pointcounts);
% function [X,Y] = gaussianclouds(centers, variances, pointcounts);
%
% Creates 2d Gaussian clouds.  Note that the returned X matrix
% has 3 columns, the last column is all ones (to be used as a
% bias variable).

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

nc = size(centers,1);

tp = sum(pointcounts);
X = zeros(tp,3);
X(:,3) = 1;

Y = repmat(-1,tp,nc);


count = 1;
for i = 1:nc
  curcount = pointcounts(i);
  G = chol(variances{i});
  centers(i,:);
  Xc = randn(curcount,2)*G + ones(curcount,1)*centers(i,:);

  curind = count:(count+curcount-1);
  X(curind,1:2) = Xc;
  Y(curind,i) = 1;

  count = curcount + count;
end
