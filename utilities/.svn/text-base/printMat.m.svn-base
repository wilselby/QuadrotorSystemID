function printMat(m,name)
disp([ name '=[']);
for i = 1:size(m,1)
    dstr = [];
    for j = 1:size(m,2)
        try
            dstr = [dstr num2str(m(i,j)) ','];
        catch
            dstr = [dstr char(m(i,j)) ','];
        end
    end
    disp(dstr);
end
disp('];');


end
