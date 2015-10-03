function ch = colorchar(i)
% ch = colorchar(i)
%
% Returns a format string for the i'th color, under matlab or octave.

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).
  
p = octaveormatlab;

if (strcmp(p,'octave'))
  ch = sprintf('%d', i);
else
  cols = 'bgrcmyk';
  ch = cols(mod(i,length(cols))+1);
end
