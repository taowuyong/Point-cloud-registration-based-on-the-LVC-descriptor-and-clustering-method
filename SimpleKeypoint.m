function [ keypoint ] = SimpleKeypoint( PC,pcloud,pr )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
keypointcloud = pcdownsample(pcloud,'gridAverage',7*pr);
keypoint=keypointcloud.Location;
[n m]=size(keypoint);
[idx dist]=rangesearch(PC,keypoint,15*pr);
nidx=[];
for i=1:n
    KNN=PC(idx{i},:);
    [h1 l1]=size(KNN);
    Cov=(1/h1)*(KNN-ones(h1,1)*mean(KNN))'*(KNN-ones(h1,1)*mean(KNN));
    [U S V]=svd(Cov);
    lam1=S(1,1);
    lam3=S(3,3);
    if h1<50 || (lam3/lam1)<0.01    %parameter
        nidx=[nidx i];
    end
end
keypoint(nidx,:)=[];
end

