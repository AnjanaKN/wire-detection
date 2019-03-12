% function [] = wire_detector(I1,I2)
disparityRange = [-8 8];
disparityMap = disparity(rgb2gray(I1),rgb2gray(I2),'BlockSize',15,'DisparityRange',disparityRange);
figure 
imshow(disparityMap,disparityRange);
title('Disparity Map');
colormap(gca,jet) 
colorbar


% end

lines = houghlines(BW,T,R,P,'FillGap',15,'MinLength',1);
figure(1), imshow(I1), hold on
max_len = 0;
for k = 1:length(lines)
   if(abs(lines(k).theta)>80)%&&abs(lines(k).theta)>70)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
   display(k);
   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
  
   end
end

figure(2);imshow(I1);hold on;plot([209;2371],[819,856],'LineWidth',2,'Color','yellow');
%=================================================================================================
cam_params = ReadYaml('left.yaml');
cam_mat = reshape(cell2mat(cam_params.camera_matrix.data), [3,3])';
cam_dist = cell2mat(cam_params.distortion_coefficients.data);
cam_skew = 0.0;

K=[cam_mat(1,1),0,0;0 cam_mat(2,2),0;cam_mat(1,3),cam_mat(2,3),1];

cam_radial = [cam_dist(1),cam_dist(2),cam_dist(5)];
cam_tang = [cam_dist(3:4)];
C1 = cameraParameters('IntrinsicMatrix', K,'RadialDistortion', cam_radial,'TangentialDistortion', cam_tang);


cam_params = ReadYaml('right.yaml');
cam_mat = reshape(cell2mat(cam_params.camera_matrix.data), [3,3])';
cam_dist = cell2mat(cam_params.distortion_coefficients.data);
cam_skew = 0.0;

K=[cam_mat(1,1),0,0;0 cam_mat(2,2),0;cam_mat(1,3),cam_mat(2,3),1];

cam_radial = [cam_dist(1),cam_dist(2),cam_dist(5)];
cam_tang = [cam_dist(3:4)];
C2 = cameraParameters('IntrinsicMatrix', K,'RadialDistortion', cam_radial,'TangentialDistortion', cam_tang);

B=-163.901008;
stereoParams = stereoParameters(C1,C2,eye(3),[B,0,0]);
%disparityMap=disparityMap(1:2021,1:2389);
% xyzPoints = reconstructScene(disparityMap,stereoParams);
 figure();
pcshow(xyzPoints)
