clc;
clear all;
close all;
fx = 462.025177*2;
fy = 462.025177*2;
cx = 323.529724*2;
cy = 187.010056*2;

imageCur=imread("color1.jpg");
%{
depthCur=imread("2depth.png");
depthCur = double(depthCur)/1000;
% plane and surface normal vector
[sNV, sPP] = estimateSurfaceNormalGradient_astra(depthCur, cam);
surfaceNormalVector = sNV;
surfacePixelPoint = sPP;
%plot_plane_image(pNV, sNV, sPP, imageCur, optsLPIC);
% refine plane normal vector
%[pNV, isTracked] = trackPlane(pNV, sNV, optsLPIC);
numTrial = 100;
ConvergeAngle = 3/180*pi;
ConeAngle = 45/180*pi;
c = 20;
minNumSample = 100;% optimal;
[MF_can, MF, FindMF] = SeekMMF(sNV,numTrial,ConvergeAngle,ConeAngle,c,minNumSample);
ratio = 0.1;
R = ClusterMMF(MF_can,ratio);
R = R{1};

% MMF seeking parameters
numTrial = 100;
ConvergeAngleforTracking = 0.01/180*pi;
ConeAngle_tracking = 3/180*pi;
c = 20;
minNumSample = 100;% optimal;
[R,IsTracked] = TrackingMF(R,sNV,ConvergeAngleforTracking,ConeAngle_tracking,c,minNumSample);
%}

%plot_planes_image_astra(R, surfaceNormalVector, surfacePixelPoint, imageCur, ConeAngle_tracking);


%%----------------------plot on Gaussian Sphere------------%%

R=[-0.925479753501830	-0.361480834785871	0.113220280606888;
-0.0111697770528338	-0.272721760476305	-0.962028106368673;
0.378632357226350	-0.891602180036150	0.248360807326766];


%%-------------------------line------------------------------%%

colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k'];
cam = [fx,  0, cx; 0, fy, cy; 0,  0,  1];
Kinv = inv(cam);

load('LineExtraction_astra3.mat');
vpCells ={vp1, vp2, vp3, vp4, vp5};

allCircleNormalcell = cell(1, size(vpCells, 2));
for k = 1:size(vpCells,2) %1~6
    for i = 1:size(vpCells{k},(1))/2 %number of line in each vp
        
        ptEnd1_p_d = [vpCells{k}(2*i-1,1:2), 1].';
        ptEnd2_p_d = [vpCells{k}(2*i  ,1:2), 1].';
        ptEnd1_n_d = Kinv * ptEnd1_p_d;
        ptEnd2_n_d = Kinv * ptEnd2_p_d;
        ptEnd1_n_d = ptEnd1_n_d / norm(ptEnd1_n_d);
        ptEnd2_n_d = ptEnd2_n_d / norm(ptEnd2_n_d);
        %plot_great_circle(ptEnd1_n_d, ptEnd2_n_d, 4, 'g', 'greatcircle');
        lines(k)
        % plot great circle normal vectors
        circleNormal = cross(ptEnd1_n_d.', ptEnd2_n_d.');
        circleNormal = circleNormal / norm(circleNormal);
        %plot3([0 circleNormal(1)],[0 circleNormal(2)],[0 circleNormal(3)], 'o', 'Color' ,colors(k),'LineWidth', 2);
        allCircleNormalcell{k}{i} = circleNormal;
   end
end


figure(3);
imshow(imageCur)
for k = 1:size(vpCells,2) %1~6
    for i = 1:size(vpCells{k},(1))/2 %number of line in each vp
        
        ptEnd1 = [vpCells{k}(2*i-1,1:2)];
        ptEnd2 = [vpCells{k}(2*i  ,1:2)];
        line([ptEnd1(1) ptEnd2(1)], [ptEnd1(2) ptEnd2(2)], 'Color', ...
            colors(k), 'LineWidth',5)
        
    end
end

VP1 = cross(allCircleNormalcell{1}{1},allCircleNormalcell{1}{2});
VP1 = VP1 / norm(VP1);
VP2 = cross(allCircleNormalcell{2}{1},allCircleNormalcell{2}{2});
VP2 = VP2 / norm(VP2);
VP3 = cross(VP1,VP2);
VP3 = VP3 / norm(VP3);
R_line = [VP1.' VP2.' VP3.'];

h1 = figure;
plot_unit_sphere(1, 18, 0.8); hold on; grid on; axis equal;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
f = FigureRotator(gca());
for k = 1:3
    plot3([0 R(1,k)],[0 R(2,k)],[0 R(3,k)], 'Color' ,'r','LineWidth', 5); hold on;
    plot3([0 R_line(1,k)],[0 R_line(2,k)],[0 R_line(3,k)], 'Color' ,'b','LineWidth', 5); hold on;
    plot3(-[0 R(1,k)],-[0 R(2,k)],-[0 R(3,k)], 'Color' ,'r','LineWidth', 5); hold on;
    plot3(-[0 R_line(1,k)],-[0 R_line(2,k)],-[0 R_line(3,k)], 'Color' ,'b','LineWidth', 5); hold on;
end 


%{
nexttile
imshow(imageCurForLine)
lines_2d = zeros(4,size(lines,1));
for k = 1:size(lines_2d,2)
    lines_2d(:,k) = lines(k,1:4); 
    line([lines_2d(1,k) lines_2d(3,k)], [lines_2d(2,k) lines_2d(4,k)], 'Color', ...
         'y', 'LineWidth',5)
        
end
%}

