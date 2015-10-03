function plotclouds(X, Y)
% A function to plot 2D point clouds.
%
% X should be n by 2 (actually, columns beyond the second are ignored), 
% and Y should be n by c.  Y should contain entirely 1's and -1's, 
% and each row of Y should contain a single 1, indicating the class 
% of the point.
%
% Assumes sizes match.  Leave the current plot held.

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

n = size(X,1);
c = size(Y,2);

for cc = 1:c
  p = find(Y(:,cc)==1);
  Xc = X(p,:);
  
  style = plotstr('points');
  fmt = sprintf('%s%s', style, colorchar(cc));
  plot(Xc(:,1),Xc(:,2), fmt);
  hold on;
end
