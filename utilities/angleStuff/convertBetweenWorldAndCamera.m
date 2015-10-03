function out = convertBetweenWorldAndCamera(in)
%convert from world back to camera coordinates
out = zeros(size(in));
for i = 1:size(in,1)
    rotM = rodrigues(in(i,4:6));
    out(i,1:3) = (rotM'*(-in(i,1:3))')';
    out(i,4:6)=-in(i,4:6);
end

end