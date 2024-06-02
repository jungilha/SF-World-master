clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;
addpath(genpath(pwd))
addpath('addon/lsd_1.6');
addpath('addon/lsd_1.6/Matlab');
addpath('MatlabProgressBar');
addpath('forDrawingFigures');

%% basic setup for LPIC

% choose the experiment case
% ICL NUIM dataset (1~8)
expCase = 6; % stairScene
colors = {'r', 'g', 'b', 'c', 'm', 'y'};
% are figures drawn?
% 1 : yes, draw figures to see current status
% 0 : no, just run LPIC
toVisualize = 1;

% are data results saved?
% 1 : yes, save the variables and results
% 0 : no, just run LPIC
toSave = 1;


setupParams_TAMU;


% load ICL NUIM dataset data
rawICLNUIMdataset = rawTAMUdataset_load(datasetPath,M);
%process_data(datasetPath);

% camera calibration parameters
[ICLNUIMdataset] = rawICLNUIMdataset; %getSyncTUMRGBDdataset(rawICLNUIMdataset, imInit, M);
optsLPIC = load_param_LPIC_TAMU;
cam = initialize_cam_TAMU(ICLNUIMdataset, optsLPIC.imagePyramidLevel);


%% load ground truth data

%{
% ground truth trajectory in ICL NUIM dataset
R_gc_true = zeros(3,3,M);
p_gc_true = zeros(3,M);
T_gc_true = cell(1,M);
for k = 1:M
    % camera body frame
    R_gc_true(:,:,k) = q2r(ICLNUIMdataset.vicon.q_gc_Sync(:,k));
    p_gc_true(:,k) = ICLNUIMdataset.vicon.p_gc_Sync(:,k);
    T_gc_true{k} = [ R_gc_true(:,:,k), p_gc_true(:,k);
        zeros(1,3),           1; ];
end
%}

%{
% if (toVisualize)
    figure; hold on; axis equal;
    L = 0.1; % coordinate axis length
    A = [0 0 0 1; L 0 0 1; 0 0 0 1; 0 L 0 1; 0 0 0 1; 0 0 L 1]';
    
    for k = 1:10:M
        T = T_gc_true{k};
        B = T * A;
        plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',1); % x: red
        plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',1); % y: green
        plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',1); % z: blue
    end
    plot3(p_gc_true(1,:),p_gc_true(2,:),p_gc_true(3,:),'k','LineWidth',2);
    
    title('ground truth trajectory of cam0 frame')
    xlabel('x'); ylabel('y'); zlabel('z');
end
%}

%{
% generate ground truth trajectory in vector form
stateTrue = zeros(6,M);
stateTrue(1:3,:) = p_gc_true;
for k = 1:size(p_gc_true,2)
    [yaw, pitch, roll] = dcm2angle(R_gc_true(:,:,k));
    stateTrue(4:6,k) = [roll; pitch; yaw];
end
%}

%% main LPIC part


% 1. Manhattan frame tracking for LPIC
systemInited_LPIC = false;

R_gc1 = [1 0 0;0 1 0;0 0 1];
R_gc_LPIC = zeros(3,3,M);
R_gc_LPIC(:,:,1) = R_gc1;


% 2. make figures to visualize current status
if (toVisualize)
    % create figure
    h = figure(10);
    set(h,'Color',[1 1 1]);
    set(h,'Units','pixels','Position',[300 20 1300 900]);
    ha1 = axes('Position',[0.05,0.55 , 0.4,0.4]);
    axis off;
    ha2 = axes('Position',[0.55,0.55 , 0.4,0.4]);
    axis off;
    ha3 = axes('Position',[0.05,0.05 , 0.4,0.4]);
    axis off;
    ha4 = axes('Position',[0.55,0.05 , 0.4,0.4]);
    grid on; hold on;
end


% 3. record vpInfo variables
vpInfo_LPIC = cell(1,M);
pNV_LPIC = cell(1,M);


% do LPIC
for imgIdx = 1:M
    
    %% 1. Manhattan frame tracking
    % image
    imageCurForLine = getImgInSTSCdataset_VGA(datasetPath, ICLNUIMdataset, cam, imgIdx, 'gray');
    imageCurForMW = getImgInSTSCdataset_VGA(datasetPath, ICLNUIMdataset, cam, imgIdx, 'rgb');
    depthCurForMW = getImgInSTSCdataset_VGA(datasetPath, ICLNUIMdataset, cam, imgIdx, 'depth');
    lineLength = optsLPIC.lineLength;
    dimageCurForLine = double(imageCurForLine);
    %[lines, ~] = lsdf(dimageCurForLine, (lineLength^2));
    load('Lines_TAMU_b.mat');
    vpCells ={vp1, vp2, vp3, vp4, vp5};
    aa = 1;
    allCircleNormalcell = cell(1, size(vpCells, 2));
    figure(3);
    imshow(imageCurForLine)
    for k = 1:size(vpCells,2) %1~6
        for i = 1:size(vpCells{k},(1))/2 %number of line in each vp
        
        ptEnd1 = [vpCells{k}(2*i-1,1:2)];
        ptEnd2 = [vpCells{k}(2*i  ,1:2)];
        line([ptEnd1(1) ptEnd2(1)], [ptEnd1(2) ptEnd2(2)], 'Color', ...
            colors{k}, 'LineWidth',2)
        lines(aa,:) = [ptEnd1 ptEnd2];
        aa = aa+1;
        end
    end

    % for the first time in this loop
    if (~systemInited_LPIC)
        % initialize and seek the dominant MF
        %[R_cM, vpInfo, pNV, sNV, sPP] = seekSanfranciscoWorld(imageCurForLine, imageCurForMW, depthCurForMW, cam, optsLPIC);
        [R_cM, R_SLP, pNV] = seekSanFranciscoWorld_MNPL_VGA(imageCurForLine, imageCurForMW, depthCurForMW, lines, cam, optsLPIC);
        
        R_c1M = R_cM(:,1:3);
        R_gM = R_gc1 * R_c1M;
        systemInited_LPIC = true;
        
        % initialize Kalman filter
        %[state] = initializeVPs(R_c1M, optsLPIC);
        
    elseif (systemInited_LPIC)
        
        % propagation step in KF
        %[state] = predictVPs(state, optsLPIC);
        R_SLP_old = R_SLP;
        % track Manhattan frame
        [R_cM, R_SLP, vpInfo, pNV, sNV, sPP, clusteredLinesIdx, maxVoteSumIdx] = trackSanFranciscoWorld_3(R_cM, pNV,R_SLP, imageCurForLine, imageCurForMW, depthCurForMW,lines, cam, optsLPIC,imgIdx);
        vpInfo_LPIC{imgIdx} = vpInfo;
        pNV_LPIC{imgIdx} = pNV;
       
    
        % correction step in KF
        %[state] = updateVPs(state, R_cM, optsLPIC);
        %[state, R_cM] = extractVPs(state);
        
        
        % update current camera pose
        R_gc_current = R_gM * inv(R_cM);
        R_gc_LPIC(:,:,imgIdx) = R_gc_current;
    end
    
    
    %% 2. update 6 DoF camera pose and visualization
    fprintf('imgIdx: %d  \n', imgIdx);
    if (imgIdx >= 46)
        % visualize current status
        plots_status_tracking2;
    else
    %   plot_save_1st_frame;
    end
    
    
end

% convert camera pose representation
stateEsti_LPIC = zeros(3,M);
for k = 1:M
    [yaw, pitch, roll] = dcm2angle(R_gc_LPIC(:,:,k));
    stateEsti_LPIC(:,k) = [roll; pitch; yaw];
end


%% plot error metric value (RPE, ATE)


% 1) LPIC rotation estimation trajectory
figure;
subplot(3,1,1);
plot(stateTrue(4,:),'k','LineWidth',2); hold on; grid on;
plot(stateEsti_LPIC(1,:),'r','LineWidth',2); hold off; axis tight; ylabel('roll (rad)');
legend('True','LPIC Matlab');
subplot(3,1,2);
plot(stateTrue(5,:),'k','LineWidth',2); hold on; grid on;
plot(stateEsti_LPIC(2,:),'r','LineWidth',2); hold off; axis tight; ylabel('pitch (rad)');
subplot(3,1,3);
plot(stateTrue(6,:),'k','LineWidth',2); hold on; grid on;
plot(stateEsti_LPIC(3,:),'r','LineWidth',2); hold off; axis tight; ylabel('yaw (rad)');


% 2) calculate rotation matrix difference
[RMD_MEAN_LPIC, RMD_LPIC] = calcRMD(R_gc_LPIC, R_gc_true);
fprintf('MEAN of RMD [deg] : %f \n' , RMD_MEAN_LPIC);
fprintf('std. of RMD [deg] : %f \n' , std(RMD_LPIC));


% 3) draw figures for rotation matrix difference
figure;
plot(RMD_LPIC, 'r*'); hold on; grid on;
curve_x = 1:M;
p = polyfit(curve_x, RMD_LPIC, 3);
curve_y = polyval(p, curve_x); curve_y(curve_y < 0) = 0;
plot(curve_x, curve_y, 'g-', 'LineWidth',5); hold off; axis tight; ylabel('RMD (deg)');
legend('RMD', 'Fitting Curve','Location','northwest');

figure;
ploterrhist(RMD_LPIC, 'bins', 25);


%% save the experiment data for CVPR 2018

if (toSave)
    save([SaveDir '/LPIC.mat']);
end

