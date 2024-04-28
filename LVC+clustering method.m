%%LVC+clustering method
str='E:\compile document\matlab\data\building point cloud\castle1-castle2.txt';
fid = fopen(str,'r');
D = textscan(fid, '%f%f%f%f');
fclose(fid);
Ttrue=[D{1} D{2} D{3} D{4}];
tic;
pcloud1=pcread('E:\compile document\matlab\data\building point cloud\castle1.ply');
PC1=pcloud1.Location;
pcloud2=pcread('E:\compile document\matlab\data\building point cloud\castle2.ply');
PC2=pcloud2.Location;
% pr=0.0263570186;        %indoor point cloud
% pr=0.1410025175;        %outdoor point cloud
pr=0.16526765;          %castle1  castle2
[n m]=size(PC1);
% plot3(PC1(:,1),PC1(:,2),PC1(:,3),'.b','MarkerSize',1);
% hold on;
% plot3(PC2(:,1),PC2(:,2),PC2(:,3),'.r','MarkerSize',1);
% set(gca,'DataAspectRatio',[1 1 1]);
% axis off
[keypoint1] = SimpleKeypoint(PC1,pcloud1,pr);
[keypoint2] = SimpleKeypoint(PC2,pcloud2,pr);
% plot3(PC1(:,1),PC1(:,2),PC1(:,3),'.b','MarkerSize',1);
% hold on;
% plot3(keypoint1(:,1),keypoint1(:,2),keypoint1(:,3),'.r','MarkerSize',5);
% set(gca,'DataAspectRatio',[1 1 1]);
% axis off
[n1 m1]=size(keypoint1);
[n2 m2]=size(keypoint2);
RR=15*pr;
[idx dist]=rangesearch(PC1,keypoint1,RR);
MV1=[];
MDV1=[];
for i=1:n1
    KNN=PC1(idx{i},:);
    d=dist{i};
    [V] = LRF_LVC(KNN,RR,d,keypoint1(i,:));
    MV1=[MV1;V];
    [DV] = LVC(KNN,keypoint1(i,:),V,RR);
    MDV1=[MDV1;DV];
end
[idx dist]=rangesearch(PC2,keypoint2,RR);
MV2=[];
MDV2=[];
for i=1:n2
    KNN=PC2(idx{i},:);
    d=dist{i};
    [V] = LRF_LVC(KNN,RR,d,keypoint2(i,:));
    MV2=[MV2;V];
    [DV] = LVC(KNN,keypoint2(i,:),V,RR);
    MDV2=[MDV2;DV];
end
[idxx distt]=knnsearch(MDV1,MDV2,'k',2);
Mmatch=[];
for i=1:n2
    if distt(i,1)/distt(i,2)<=0.9
        match=[i idxx(i,1)];
        Mmatch=[Mmatch;match];
    end
end
[n3 m3]=size(Mmatch);
keypointmm2=keypoint2(Mmatch(:,1),:);
keypointmm1=keypoint1(Mmatch(:,2),:);
%%集群算法
MCo=[];
PCb=mean(PC2);
for i=1:n3
    keypoint22=keypoint2(Mmatch(i,1),:);
    V2=MV2((3*Mmatch(i,1)-2):3*Mmatch(i,1),:);
    calpha=(PCb-keypoint22)*V2(:,1)/norm(PCb-keypoint22);
    cbate=(PCb-keypoint22)*V2(:,2)/norm(PCb-keypoint22);
    cgama=(PCb-keypoint22)*V2(:,3)/norm(PCb-keypoint22);
    keypoint11=keypoint1(Mmatch(i,2),:);
    V1=MV1((3*Mmatch(i,2)-2):3*Mmatch(i,2),:);
    vector=[calpha cbate cgama]*inv(V1);
    Co=keypoint11+vector*norm(PCb-keypoint22);
    MCo=[MCo;Co];
end
[idxCo distCo]=rangesearch(MCo,MCo,5*pr);        %%parameter
clear neinum;
for i=1:n3
    neinum(i)=length(idxCo{i});
end
[ma id]=max(neinum);
PidxCo=idxCo{id};
cmatch=Mmatch(PidxCo,:);
keypointm2=keypoint2(cmatch(:,1),:);
keypointm1=keypoint1(cmatch(:,2),:);
[n6 m6]=size(keypointm2);
[n7 m7]=size(keypointmm2);
cMV2=[];
cMV1=[];
for i=1:n6
    V2=MV2((3*cmatch(i,1)-2):3*cmatch(i,1),:);
    V1=MV1((3*cmatch(i,2)-2):3*cmatch(i,2),:);
    cMV2=[cMV2;V2'];
    cMV1=[cMV1;V1'];
end
[U S V]=svd(cMV1'*cMV2);
D=diag([1 1 det(U*V')]);
R=V*D*U';
t=mean(keypointm2,1)-mean(keypointm1,1)*R';
PC1t=PC1*R'+ones(n,1)*t;
time=toc;
T=[R t';zeros(1,3) 1];
errorR=real(acos((trace(Ttrue(1:3,1:3)*inv(R))-1)/2)*(180/pi));
errort=norm(Ttrue(1:3,4)-t');
errorR
errort
time
n6
% subplot(1,3,1);
% plot3(PC2(:,1),PC2(:,2),PC2(:,3),'.r','MarkerSize',1);
% hold on;
% plot3(PC1(:,1),PC1(:,2),PC1(:,3),'.b','MarkerSize',1);
% hold on;
% for i=1:n7
%     line([keypointmm2(i,1),keypointmm1(i,1)],[keypointmm2(i,2),keypointmm1(i,2)],[keypointmm2(i,3),keypointmm1(i,3)],'linewidth',1,'color','k');
% end
% set(gca,'DataAspectRatio',[1 1 1]);
% axis off
% subplot(1,3,2);
% plot3(PC2(:,1),PC2(:,2),PC2(:,3),'.r','MarkerSize',1);
% hold on;
% plot3(PC1(:,1),PC1(:,2),PC1(:,3),'.b','MarkerSize',1);
% hold on;
% for i=1:n6
%     line([keypointm2(i,1),keypointm1(i,1)],[keypointm2(i,2),keypointm1(i,2)],[keypointm2(i,3),keypointm1(i,3)],'linewidth',1,'color','k');
% end
% set(gca,'DataAspectRatio',[1 1 1]);
% axis off
% subplot(1,3,3);
plot3(PC1t(:,1),PC1t(:,2),PC1t(:,3),'.b','MarkerSize',1);
hold on;
plot3(PC2(:,1),PC2(:,2),PC2(:,3),'.r','MarkerSize',1);
set(gca,'DataAspectRatio',[1 1 1]);
axis off