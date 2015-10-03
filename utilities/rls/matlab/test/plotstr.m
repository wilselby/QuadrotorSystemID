function s = plotstring(style)
% ch = colorchar(i)
%
% Returns a format string for the style, under matlab or octave.
% Styles supported: points

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).
  
p = octaveormatlab;

if (strcmp(style, 'points'))
  if (strcmp(p, 'octave'))
    s = '@';
  else
    s = '.';
  end
 
else
  error('Unknown style in plotstr');
end
