function [ V ] = LRF_LVC( KNN,RR,d,keypoint )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
[h l]=size(KNN);
idc=find(d<(1/3)*RR);
KNNc=KNN(idc,:);
[h1 l1]=size(KNNc);
centroid=mean(KNNc);
[U S V]=svd((KNNc-ones(h1,1)*centroid)'*(KNNc-ones(h1,1)*centroid));
qp=ones(h,1)*keypoint-KNN;
qp=sum(qp);
if qp*V(:,3)>=0
    V3=V(:,3);
else
    V3=-V(:,3);
end
Vertical=[0;0;1];
V1=cross(V3,Vertical);
V1=V1/norm(V1);
V2=cross(V1,V3);
V=[V1 V2 V3];
end

