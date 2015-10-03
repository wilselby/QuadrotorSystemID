function [V, D] = rls_vd(K);
% Performs an eigendecomposition of K and returns a matrix
% and a row vector D of eigenvalues, s.t. V*D*V' = K. 

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

[V,D] = eig(K);
D = diag(D)';   % Want a row
