function [rawICLNUIMdataset] = rawSTSCdataset_load(datasetPath,length)
%% import data from text files

%{
delimiter = ' ';
% import ground-truth data
if exist([datasetPath '/groundtruth.txt'],'file') 
    textData_groundtruth = importdata([datasetPath '/groundtruth.txt'], delimiter);
else
    textData_groundtruth = readmatrix([datasetPath '/odometry.csv'],'NumHeaderLines',1,'Range','B:I');
end
%}

% create 'association.txt'
if exist([datasetPath '/associations.txt'], 'file') == 2
    disp('association.txt has already been made.');
else
    fileID = fopen([datasetPath '/associations.txt'],'w');
    for i = 1:length
        str = sprintf('%d depth_new/%d.png %d rgb_new/%d.png %d confidence_new/%d.png\n', i-1, i-1, i-1, i-1, i-1, i-1);
        fprintf(fileID, str);
    end
    fclose(fileID);
    disp('Generated associations.txt.');
end

% import time_sync (rgb and depth) data
fileID = fopen([datasetPath, '/associations.txt'], 'r');
textData_timesync = textscan(fileID, '%f64 %s %f64 %s %f64 %s');
fclose(fileID);

%{
%% load ground-truth and rgb data

% load ground-truth data
rawICLNUIMdataset.vicon.time = textData_groundtruth(:,1).';
rawICLNUIMdataset.vicon.p_gc = [textData_groundtruth(:,2).'; textData_groundtruth(:,3).'; textData_groundtruth(:,4).'];
rawICLNUIMdataset.vicon.q_gc = [textData_groundtruth(:,8).'; textData_groundtruth(:,5).'; textData_groundtruth(:,6).'; textData_groundtruth(:,7).'];
% [qw qx qy qz]

% convert camera frame (to my convention)
q_gc_temp = rawICLNUIMdataset.vicon.q_gc;
numData = size(q_gc_temp, 2); %q_gc_temp의 2번쨰 요소 사이즈
q_gc_convert = zeros(4, numData);
for k = 1:numData
    R_gc_true = q2r(q_gc_temp(:, k));
    angle_true = rotmtx2angle(R_gc_true.'); %[phi;theta;psi]
    
    angle_true_convert = [angle_true(1); angle_true(2); angle_true(3)]; %[-phi;theta;-psi]
    R_gc_true_convert = angle2rotmtx(angle_true_convert).';
    
    q_gc_convert(:,k) = r2q(R_gc_true_convert);
end
rawICLNUIMdataset.vicon.q_gc = q_gc_convert;
%}

% load synchronized rgb data
rawICLNUIMdataset.rgb.Ratio= 1;
rawICLNUIMdataset.rgb.dRatio = 1;
%intrinsics_path = [datasetPath '/camera_matrix.csv'];
K = [525 0 319.5 ; 0 525 239.5 ; 0 0 1.0];
color_K = eye(3);
color_K(1,:) = K(1,:) / rawICLNUIMdataset.rgb.Ratio;
color_K(2,:) = K(2,:) / rawICLNUIMdataset.rgb.Ratio;
depth_K = eye(3);
depth_K(1,:) = K(1,:) / rawICLNUIMdataset.rgb.dRatio;
depth_K(2,:) = K(2,:) / rawICLNUIMdataset.rgb.dRatio;
save([datasetPath '/camera_matrix.mat'],'color_K','depth_K');

distortion = [0, 0, 0, 0, 0];  % d0 d1 d2 d3 d4
 

rawICLNUIMdataset.rgb.time = textData_timesync{3}.';
rawICLNUIMdataset.rgb.imgName = textData_timesync{4}.';
rawICLNUIMdataset.rgb.K = color_K;
rawICLNUIMdataset.rgb.KD = depth_K;
rawICLNUIMdataset.rgb.distortion = distortion;


% load synchronized depth data
rawICLNUIMdataset.depth.time = textData_timesync{1}.';
rawICLNUIMdataset.depth.imgName = textData_timesync{2}.';
rawICLNUIMdataset.depth.scaleFactor = 1000;

% load synchronized confidence data
rawICLNUIMdataset.confi.time = textData_timesync{5}.';
rawICLNUIMdataset.confi.imgName = textData_timesync{6}.';


end
