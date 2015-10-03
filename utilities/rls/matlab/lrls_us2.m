function [U, S2] = lrls_us2(X);
% Performs the SVD of X and returns U and a row vector
% with the squared singular values.

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

[U,S,V] = svd(X,0);
S2 = ((diag(S)).^2)';   % Want a row

