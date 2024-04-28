# Point-cloud-registration-based-on-the-LVC descriptor-and-clustering-method
In this project, we provide the Matlab code of a registration based on the LVC descriptor. The method first uses the simple keypoint detector to extract keypoints. Then, the LVC descriptor is calculated for each keypoint. By comparing the similarity of the descriptors, the correspondences are established. Finally, the clustering method is used to find the correct correspondences and estimate the transformation.

Everyone is welcome to use the code for research work, but not for commerce. If you use the code, please cite my paper (Wuyong Tao, Tieding Lu, Xijiang Chen, Zhiping Chen, Wei Li, Meng Pang. A Local Shape Descriptor Designed for Registration of Terrestrial Point Clouds. IEEE Transactions on Geosciences and Remote Sensing.)

In this project, four files are provided. The “SimpleKeypoint” file is used to extracted the keypoints. The “LRF_LVC” file is used to calculate the LRF of the descriptor. The “LVC” is applied to calculate the LVC descriptor. The “LVC+clustering method” file performs the point cloud registration based on the LVC descriptor and clustering method. 

Before you carry out our algorithm, you need to calculate the point cloud resolution (pr).
