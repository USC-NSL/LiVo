clear all;
close all;
clc;

% taskset --cpu-list 3-5 matlab -nodisplay -nosplash -nodesktop -r "addpath(genpath('/home/lei/rajrup/KinectStream/Metrics/pointssim'));"

START_FRAME=34;
END_FRAME=10000;
SEQ_NAME='170307_dance5_with_ground';
CONFIG_FILE='/home/lei/rajrup/KinectStream/Multiview/config/panoptic_170307_dance5.json';

% culling_method = "o3d_nvenc_s_kpcull_c_cull";
culling_method = "o3d_nvenc_s_pcull_c_cull";

trace_type='tracep1'; % Used tracep1 for this

config = jsondecode(fileread(CONFIG_FILE));
logID = int32(config.log_id);

original_path = sprintf('/datassd/pipeline_ptcl/client_ptcl/o3d_gt/%s/o3d_gt_with_ground_c_cull/log%d/', SEQ_NAME, logID);
distorted_path = sprintf('/datassd/pipeline/client_tiled/ablation/%s/%s/%s/log%d/', SEQ_NAME, culling_method, trace_type, logID);
comp_path=sprintf('/datassd/pipeline/client_tiled/ablation/%s/', SEQ_NAME);
out_file_path=sprintf('%s/3D_pssim_%s_%s_%s_log%d.csv', comp_path, SEQ_NAME, culling_method, trace_type, logID);

fprintf('File path: %s\n', out_file_path);
metrics_file = fopen(out_file_path, 'w');
fprintf(metrics_file, 'Frame,PSSIM_GEO_AB,AVG_PSSIM_GEO_AB,PSSIM_GEO_BA,AVG_PSSIM_GEO_BA,PSSIM_COLOR_AB,AVG_PSSIM_COLOR_AB,PSSIM_COLOR_BA,AVG_PSSIM_COLOR_BA\n');

start_frame_id = START_FRAME;
end_frame_id = END_FRAME;

%% Configurations
PARAMS.ATTRIBUTES.GEOM = true;
PARAMS.ATTRIBUTES.NORM = false;
PARAMS.ATTRIBUTES.CURV = false;
PARAMS.ATTRIBUTES.COLOR = true;

% PARAMS.ESTIMATOR_TYPE = {'VAR', 'Mean'};
PARAMS.ESTIMATOR_TYPE = {'Mean'};
PARAMS.POOLING_TYPE = {'Mean'};
PARAMS.NEIGHBORHOOD_SIZE = 12;
PARAMS.CONST = eps(1);
PARAMS.REF = 0;

FITTING.SEARCH_METHOD = 'rs';
if strcmp(FITTING.SEARCH_METHOD, 'rs')
    ratio = 0.01;
elseif strcmp(FITTING.SEARCH_METHOD, 'knn')
    knn = 12;
end
FITTING.SEARCH_SIZE = [];

QUANT.VOXELIZATION = false;
QUANT.TARGET_BIT_DEPTH = 9;

pssim_geomAB_list = [];
pssim_colorAB_list = [];
pssim_geomBA_list = [];
pssim_colorBA_list = [];
for frameID = 0:100:end_frame_id

    %% Load point clouds
    if isfile([original_path, num2str(frameID), '.ply']) == 0
        fprintf('File %s does not exist\n', [original_path, num2str(frameID), '.ply']);
        fprintf("Skipping frame %d\n", frameID)
        continue;
    end

    if isfile([distorted_path, num2str(frameID), '.ply']) == 0
        fprintf('File %s does not exist\n', [distorted_path, num2str(frameID), '.ply']);
        fprintf("Skipping frame %d\n", frameID)
        continue;
    end
    
    A = pcread([original_path, num2str(frameID), '.ply']);
    B = pcread([distorted_path, num2str(frameID), '.ply']);

    %% Sort geometry
    [geomA, idA] = sortrows(A.Location);
    if ~isempty(A.Color)
        colorA = A.Color(idA, :);
        A = pointCloud(geomA, 'Color', colorA);
    else
        A = pointCloud(geomA);
    end

    [geomB, idB] = sortrows(B.Location);
    if ~isempty(B.Color)
        colorB = B.Color(idB, :);
        B = pointCloud(geomB, 'Color', colorB);
    else
        B = pointCloud(geomB);
    end


    %% Point fusion
    A = pc_fuse_points(A);
    B = pc_fuse_points(B);


    %% Voxelization
    if QUANT.VOXELIZATION
        A = pc_vox_scale(A, [], QUANT.TARGET_BIT_DEPTH);
        B = pc_vox_scale(B, [], QUANT.TARGET_BIT_DEPTH);
    end


    %% Normals and curvatures estimation
    if PARAMS.ATTRIBUTES.NORM || PARAMS.ATTRIBUTES.CURV
        if strcmp(FITTING.SEARCH_METHOD, 'rs')
            FITTING.SEARCH_SIZE = round(ratio * double(max(max(A.Location) - min(A.Location))));
        else
            FITTING.SEARCH_SIZE = knn;
        end
        [normA, curvA] = pc_estimate_norm_curv_qfit(A, FITTING.SEARCH_METHOD, FITTING.SEARCH_SIZE);
        [normB, curvB] = pc_estimate_norm_curv_qfit(B, FITTING.SEARCH_METHOD, FITTING.SEARCH_SIZE);
    end


    %% Set custom structs with required fields
    sA.geom = A.Location;
    sB.geom = B.Location;
    if PARAMS.ATTRIBUTES.NORM
        sA.norm = normA;
        sB.norm = normB; 
    end
    if PARAMS.ATTRIBUTES.CURV
        sA.curv = curvA;
        sB.curv = curvB;
    end
    if PARAMS.ATTRIBUTES.COLOR
        sA.color = A.Color;
        sB.color = B.Color;
    end


    %% Compute structural similarity scores
    [pssim] = pointssim(sA, sB, PARAMS);
    pssim_geomAB = pssim.geomAB * 100.0;
    pssim_colorAB = pssim.colorAB * 100.0;
    pssim_geomBA = pssim.geomBA * 100.0;
    pssim_colorBA = pssim.colorBA * 100.0;

    pssim_geomAB_list = [pssim_geomAB_list, pssim_geomAB];
    pssim_colorAB_list = [pssim_colorAB_list, pssim_colorAB];
    pssim_geomBA_list = [pssim_geomBA_list, pssim_geomBA];
    pssim_colorBA_list = [pssim_colorBA_list, pssim_colorBA];

    avg_pssim_geomAB = mean(pssim_geomAB_list);
    avg_pssim_colorAB = mean(pssim_colorAB_list);
    avg_pssim_geomBA = mean(pssim_geomBA_list);
    avg_pssim_colorBA = mean(pssim_colorBA_list);

    fprintf(metrics_file, '%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n', frameID, pssim_geomAB, avg_pssim_geomAB, pssim_geomBA, avg_pssim_geomBA, pssim_colorAB, avg_pssim_colorAB, pssim_colorBA, avg_pssim_colorBA);

    fprintf('Frame %d: PSSIM_GEO_BA:%.2f, AVG_PSSIM_GEO_BA:%.2f, PSSIM_COLOR_BA:%.2f, AVG_PSSIM_COLOR_BA:%.2f\n', frameID, pssim_geomBA, avg_pssim_geomBA, pssim_colorBA, avg_pssim_colorBA);
    fprintf('Frame %d: PSSIM_GEO_AB:%.2f, AVG_PSSIM_GEO_AB:%.2f, PSSIM_COLOR_AB:%.2f, AVG_PSSIM_COLOR_AB:%.2f\n', frameID, pssim_geomAB, avg_pssim_geomAB, pssim_colorAB, avg_pssim_colorAB);
end