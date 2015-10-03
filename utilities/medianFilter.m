function out = medianFilter(in,windowSize)
out = zeros(size(in));
for i = windowSize+1:length(in)-windowSize
    out(i,:) = median(in(i-windowSize:i+windowSize,:));
end

out(1:windowSize,:)=repmat(out(windowSize+1,:),windowSize,1);
out(end-windowSize+1:end,:)=repmat(out(end-windowSize,:),windowSize,1);
