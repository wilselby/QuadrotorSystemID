function [d imFnames]=model2()
full_fname = 'model2.mat';
fname = 'model2.mat';
if (exist(full_fname,'file'))
    filename = full_fname;
else
    filename = fname;
end
d = load(filename);
