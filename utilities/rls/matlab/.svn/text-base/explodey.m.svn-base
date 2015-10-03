function yex = explodey(y,allowEmpty)
% function yex = explodey(Y)
% If y is an n-vector containing the integers 1..k, returns an n by k
% matrix yex satisfying yex(i,j) = 1 if y(i) = j, -1 otherwise
% Checks assumptions on y 
  
% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

  low = min(y);
  top = max(y);
  cl = max(y)-min(y)+1;
      
  if (norm(y - round(y)) > eps)
    error('rls:explodey', 'Class vector seems nonintegral');
  end

  if (length(unique(y)) ~= cl && allowEmpty ~= 1)
    error('rls:explodey', 'Missing classes in y');
  end
  
  yex = repmat(-1, length(y), cl);
  
  for c = 1:cl
    yex(y == c, c) = 1;
  end
