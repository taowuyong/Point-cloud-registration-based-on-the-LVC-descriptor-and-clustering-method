function [ DV ] = LVC( KNN,keypoint,V,RR)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[h l]=size(KNN);
KNNt=(KNN-ones(h,1)*keypoint)*V;
wbin=10;              %%²ÎÊý
Lbin=2*RR/wbin;
DV=[];
for i=1:wbin
    for j=1:wbin
        for k=1:wbin
            idd=find(KNNt(:,1)>=-RR+(i-1)*Lbin & KNNt(:,1)<-RR+i*Lbin & KNNt(:,2)>=-RR+(j-1)*Lbin & KNNt(:,2)<-RR+j*Lbin & KNNt(:,3)>=-RR+(k-1)*Lbin & KNNt(:,3)<-RR+k*Lbin);
            if length(idd)==0
                D=0;
            else
                vert=[-RR+(i-0.5)*Lbin -RR+(j-0.5)*Lbin -RR+(k-0.5)*Lbin];
                D=norm(vert)/RR;
            end
            DV=[DV D];
        end
    end
end
end