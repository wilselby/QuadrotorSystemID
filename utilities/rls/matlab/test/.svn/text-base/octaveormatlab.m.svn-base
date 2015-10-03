function whichone = octaveormatlab()
% whichone = octaveormatlab()
%
% Determines whether the script is running under octave or matlab.
% Assumes that OCTAVE_VERSION throws an error in matlab.

% Copyright rif 2006, modified BSD license (see rls/matlab/LICENSE).

try
  octave = OCTAVE_VERSION;
  whichone = 'octave';
catch
  whichone = 'matlab';
end
