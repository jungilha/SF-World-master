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


setupParams_ARkit;


% load ICL NUIM dataset data
rawICLNUIMdataset = rawSTSCdataset_load(datasetPath,M);
process_data(datasetPath);

% camera calibration parameters
[ICLNUIMdataset] = getSyncTUMRGBDdataset(rawICLNUIMdataset, imInit, M);
optsLPIC = load_param_LPIC;
cam = initialize_cam_STSC(ICLNUIMdataset, optsLPIC.imagePyramidLevel);

%% load ground truth data


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

% generate ground truth trajectory in vector form
stateTrue = zeros(6,M);
stateTrue(1:3,:) = p_gc_true;
for k = 1:size(p_gc_true,2)
    [yaw, pitch, roll] = dcm2angle(R_gc_true(:,:,k));
    stateTrue(4:6,k) = [roll; pitch; yaw];
end


%% main LPIC part

startAt = optsLPIC.startIdx;
% 1. Manhattan frame tracking for LPIC
systemInited_LPIC = false;

R_gc1 = R_gc_true(:,:,startAt);
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
for imgIdx = startAt:M
    %% 1. Manhattan frame tracking
    if imgIdx == 1098
        fprintf("stop at 57");
    end
    if(imgIdx == 1081 || imgIdx == 2876) %L1 1
    %if(imgIdx == 126 || imgIdx == 141 || imgIdx == 143 || imgIdx == 145 || imgIdx == 147 || imgIdx == 149 || imgIdx == 151 || imgIdx == 153 || imgIdx == 155  || imgIdx == 157  || imgIdx == 159 || imgIdx == 160 || imgIdx == 161 || imgIdx == 163  || imgIdx == 166 || imgIdx == 167 || imgIdx == 168 || imgIdx == 169 || imgIdx == 170 || imgIdx == 171 || imgIdx == 172 || imgIdx == 174 || imgIdx == 175 || imgIdx == 177 || imgIdx == 178 || imgIdx == 180 || imgIdx == 184 || imgIdx == 186 || imgIdx == 192 || imgIdx == 373 || imgIdx == 385 || imgIdx == 388)
        continue;
    end
    % image
    
    imageCurForLine = getImgInSTSCdataset(datasetPath, ICLNUIMdataset, cam, imgIdx, 'gray');
    imageCurForMW = getImgInSTSCdataset(datasetPath, ICLNUIMdataset, cam, imgIdx, 'rgb');
    depthCurForMW = getImgInSTSCdataset(datasetPath, ICLNUIMdataset, cam, imgIdx, 'depth');
    confiCurForMW = getImgInSTSCdataset(datasetPath, ICLNUIMdataset, cam, imgIdx, 'confi');
    depthCurForMW(confiCurForMW == 0 ) = 0;
    depthCurForMW(confiCurForMW == 1 ) = 0;
    lineLength = optsLPIC.lineLength;
    dimageCurForLine = double(imageCurForLine);
    %[lines, ~] = lsdf(dimageCurForLine, (lineLength^2));
    %{
    if(imgIdx == 27)
    newRow = [6.18261376896066 1349.60501750292 727.871353558927 1245.53821470245 0 0 0]; %L1 2
    lines(end+1, :) = newRow;
    end
    if(imgIdx == 144)
    newRow = [823.810676779463	1361.10997666278 1154.62164527421	702.209159859977 0 0 0;
                1439.37981330222	522.226954492415  1899.77654609102	530.628354725787 0 0 0]; %L1 2
    lines(end+1:end+2, :) = newRow;
    end
    if(imgIdx == 146)
    newRow = [830.728996499417	1341.83109684948 1147.01108518086	713.778879813302 0 0 0;
1402.92561260210	522.476954492415 1913.88710618436	532.238914819136 0 0 0]; %L1 2
    lines(end+1:end+2, :) = newRow;
    end
    if(imgIdx == 148)
    newRow = [831.269836639440	1352.69165694282 1165.38360560093	686.824679113185 0 0 0;
1180.61668611435	663.370478413069 1343.60385064177	612.962077012835 0 0 0]; %L1 2
    lines(end+1:end+2, :) = newRow;
    end
    if(imgIdx == 150)
    newRow = [831.269836639440	1352.69165694282 1165.38360560093	686.824679113185 0 0 0;
1180.61668611435	663.370478413069 1343.60385064177	612.962077012835 0 0 0]; %L1 2
    lines(end+1:end+2, :) = newRow;
    end
    if(imgIdx == 152)
    newRow = [885.759918319720	1340.12689614936 1160.91248541424	759.588681446908 0 0 0]; %L1 2
    lines(end+1, :) = newRow;
    end
    if(imgIdx == 154)
    newRow = [1149.02304550758	816.321761960327  884.411318553092	1387.56417736289 0 0 0]; %L1 2
    lines(end+1, :) = newRow;
    end
    if(imgIdx == 156)
    newRow = [876.452158693115	1431.20361726955  1143.22024504084	842.590723453909 0 0 0]; %L1 2
    lines(end+1, :) = newRow;
    end
    if(imgIdx == 160)
    newRow = [876.038798133022	1376.23249708285  1120.97520420070	823.311843640607 0 0 0]; %L1 2
    lines(end+1, :) = newRow;
    end
    if(imgIdx == 167)
    newRow = [814.967036172696	1434.82817969662  1050.38652275379	878.352683780630 0 0 0]; %L1 2
    lines(end+1, :) = newRow;
    end
    if(imgIdx == 169)
    newRow = [811.676196032672	1425.21761960327  1028.74008168028	906.203325554259 0 0 0]; %L1 2
    lines(end+1, :) = newRow;
    end
    %}
    %lines = extractUniqueLines(lines, cam);

    
    load('Lines_L.mat');
    vpCells ={vp1, vp2, vp3, vp4, vp5, vp6};
    aa = 1;
    allCircleNormalcell = cell(1, size(vpCells, 2));
    figure(3);
    imshow(imageCurForLine)
    for k = 1:size(vpCells,2) %1~6
        for i = 1:size(vpCells{k},(1))/2 %number of line in each vp  
        ptEnd1 = [vpCells{k}(2*i-1,1:2)];
        ptEnd2 = [vpCells{k}(2*i  ,1:2)];
        line([ptEnd1(1) ptEnd2(1)], [ptEnd1(2) ptEnd2(2)], 'Color', ...
            colors{k}, 'LineWidth',5)
        lines(aa,:) = [ptEnd1 ptEnd2 0 0 0];
        aa = aa+1;
        end
    end
    %}
    %confiCurForMW = getImgInSTSCdataset(datasetPath, ICLNUIMdataset, cam, imgIdx, 'conf');
    [imageCurForMW, depthCurForMW] = getImgPyramid(imageCurForMW, depthCurForMW, optsLPIC.imagePyramidLevel);
    
    
    % for the first time in this loop
    if (~systemInited_LPIC)
        % initialize and seek the dominant MF
        %[R_cM, vpInfo, pNV, sNV, sPP] = seekSanfranciscoWorld(imageCurForLine, imageCurForMW, depthCurForMW, cam, optsLPIC);
        [R_cM, R_SLP, pNV] = seekSanFranciscoWorld_MNPL(imageCurForLine, imageCurForMW, depthCurForMW, lines, cam, optsLPIC);
        
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
        if(optsLPIC.LShaped == 0)
        [R_cM, R_SLP, vpInfo, pNV, sNV, sPP, clusteredLinesIdx, maxVoteSumIdx] = trackSanFranciscoWorld_2(R_cM, pNV,R_SLP, imageCurForLine, imageCurForMW, depthCurForMW,lines, cam, optsLPIC, imgIdx);
        else
        [R_cM, R_SLP, vpInfo, pNV, sNV, sPP, clusteredLinesIdx, maxVoteSumIdx] = trackSanFranciscoWorld_LShaped(R_cM, pNV,R_SLP, imageCurForLine, imageCurForMW, depthCurForMW,lines, cam, optsLPIC, imgIdx);
        end
        vpInfo_LPIC{imgIdx} = vpInfo;
        pNV_LPIC{imgIdx} = pNV;
        
        
    
        % correction step in KF
        %[state] = updateVPs(state, R_cM, optsLPIC);
        %[state, R_cM] = extractVPs(state);
        
        
        % update current camera pose
        [U,~,V] = svd([R_cM(:,1), R_cM(:,2), R_cM(:,3)]);
        R_cM = U * V';
        R_gc_current = R_gM * inv(R_cM);
        R_gc_LPIC(:,:,imgIdx) = R_gc_current;
    end
    
    
    %% 2. update 6 DoF camera pose and visualization
    fprintf('imgIdx: %d  \n', imgIdx);
    if (imgIdx >= startAt+1)
        % visualize current status
        plots_status_tracking;
    else
       plot_save_1st_frame;
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

