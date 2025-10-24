clear all;
close all;
clc;

% taskset --cpu-list 0-2 matlab -nodisplay -nosplash -nodesktop -r "addpath(genpath('/home/lei/rajrup/KinectStream/Metrics/pointssim'));"

START_FRAME=1;
END_FRAME=10000;
SEQ_ID=160906;
SEQ_NAME_SHORT='pizza1';

SEQ_NAME=sprintf('%d_%s_with_ground', SEQ_ID, SEQ_NAME_SHORT);
CONFIG_FILE=sprintf('/home/lei/rajrup/KinectStream/Multiview/config/panoptic_%d_%s.json', SEQ_ID, SEQ_NAME_SHORT);

mm_type='tracep1-scaled10.0';
% mm_type='wifi-25-scaled15.0';

% pipeline_method='livo';
% culling='s_kpcull10_c_cull';

% pipeline_method='livo_nocull';
% culling='s_nocull_c_cull';

% pipeline_method='starline';
% culling='s_nocull_c_cull';

% pipeline_method='draco';
% culling='s_pcull_c_cull';

pipeline_method='meshreduce';
culling='s_nocull_c_nocull';
% culling='s_nocull_c_nocull_wo_sampling';

config = jsondecode(fileread(CONFIG_FILE));
logID = int32(config.log_id);

base_path = '/datassd';
original_path = sprintf('%s/pipeline_ptcl/client_ptcl/o3d_gt/%s/o3d_gt_with_ground_c_cull/log%d/', base_path, SEQ_NAME, logID);
distorted_path = '';
comp_path='';
out_file_path='';

if strcmp(pipeline_method, 'livo') || strcmp(pipeline_method, 'livo_nocull') || strcmp(pipeline_method, 'starline')
    distorted_path = sprintf('%s/pipeline_cpp/client_tiled/e2e_quality/%s/%s/%s/o3d_nvenc_%s/log%d/', base_path, pipeline_method, SEQ_NAME, mm_type, culling, logID);
    comp_path = sprintf('%s/pipeline_cpp/client_tiled/e2e_quality/%s/%s/%s/comp_o3d_pointssim_%s/log%d/', base_path, pipeline_method, SEQ_NAME, mm_type, culling, logID);
    out_file_path = sprintf('%s/3D_pssim.csv', comp_path);

elseif strcmp(pipeline_method, 'meshreduce')
    base_path = '/hdd';
    original_path = sprintf('%s/pipeline_ptcl/client_ptcl/o3d_gt/%s/o3d_gt_with_ground_c_nocull/', base_path, SEQ_NAME);
    distorted_path = sprintf('%s/pipeline_meshreduce/e2e_quality/mesh_to_ptcl/%s/%s/draco_nvenc_%s/', base_path, SEQ_NAME, mm_type, culling);
    comp_path = sprintf('%s/pipeline_meshreduce/e2e_quality/mesh_to_ptcl/%s/%s/comp_draco_pointssim_%s/', base_path, SEQ_NAME, mm_type, culling);
    out_file_path = sprintf('%s/3D_pssim.csv', comp_path);
    
elseif strcmp(pipeline_method, 'draco')
    if strcmp(mm_type, 'tracep1')
        mm_type_mod = 'tracep1-scaled10.0';
    elseif strcmp(mm_type, 'wifi-25')
        mm_type_mod = 'wifi-25-scaled15.0';
    else
        fprintf('Invalid mm_type\n');
        return;
    end

    distorted_path = sprintf('%s/pipeline_ptcl/client_ptcl/%s/%s/%s/log%d/', base_path, pipeline_method, SEQ_NAME, mm_type_mod, logID);
    comp_path = sprintf('%s/pipeline_ptcl/client_ptcl/%s/', base_path, pipeline_method);
    out_file_path = sprintf('%s/3D_pssim_%s_%s_%s_%s_log%d.csv', comp_path, SEQ_NAME, pipeline_method, culling, mm_type, logID);
    
else
    fprintf('Invalid pipeline_method\n');
    return;
end

if ~exist(comp_path, 'dir')
    mkdir(comp_path);
end

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

for frameID = start_frame_id:end_frame_id

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