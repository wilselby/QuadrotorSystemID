function out = applyRotFit(in, R, T)

out  = zeros(size(in));
for i = 1:size(in,1)
    Rc = rodrigues(in(i,4:6));
    out(i,1:3) = in(i,1:3)*R+T;
    try
        out(i,4:6) = rodrigues(Rc*R)';
    catch
       disp(['rodrigues had problems in ' num2str(i)]); 
    end
end

end
