function inds = findLatestsInds(t1,t2) %%%Return the indices of the most recent timestamp in t2 for each timeStamp in t1

if nnz(diff(t1)<0)>0 || nnz(diff(t2)<0)
    disp('out of order timestamps!');
end

inds = zeros(size(t1));
minInd = 1;
maxInd  = 1;
for i = 1:length(t1)
    %     ind = find(t2<=t1(i),1,'last');
    %     if (~isempty(ind))
    %         inds(i) = ind;
    %     else
    %         inds(i) = 1;
    %     end
    if (t2(1)>t1(i))
        ind2=minInd;
        %         disp('first')
    elseif (t2(end)<t1(i))
        ind2=length(t2);
        %         disp('end')
    else
        step =1;
        while (t2(maxInd)<t1(i))
            maxInd=maxInd+step;
            step=step*2;
            if (maxInd>length(t2))
                maxInd  = length(t2);
                break;
            end
            
        end
        while true
            ti = floor((maxInd+minInd)/2);
            if (t2(ti)>t1(i))
                maxInd=ti;
            else
                minInd=ti;
            end
            if (maxInd-minInd<=1)
                break;
                %         else
                %             fprintf(1,'%d %d\n',minInd,maxInd);
            end
        end
        ind2 = minInd;
        %         disp('mid')
    end
    %     if (ind2~=ind)
    %         keyboard;
    %     end
    inds(i)=ind2;
end


end