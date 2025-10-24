clear all;
close all;
clc;

original_path = "/datassd/pipeline/client_tiled/pipeline_new/160317_moonbaby1_with_ground/o3d_gt_with_ground_s_nocull_c_nocull";
distorted_path = "/datassd/pipeline_ptcl/client_ptcl/160317_moonbaby1_with_ground/o3d_ptcl_draco_wo_cull";

out_file_path = sprintf('%s/3D_pssim.csv', distorted_path);
metrics_file = fopen(out_file_path, 'w');
fprintf(metrics_file, 'Frame,DRACO_CL,DRACO_QP,PSSIM_GEO_AB,PSSIM_GEO_BA,PSSIM_COLOR_AB,PSSIM_COLOR_BA\n');

start_frame = 1000;
end_frame = 6000;

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

for frameID = start_frame:1000:end_frame
    for cl = 0:9
        for qp = 0:30
            gt_fpath = sprintf('%s/%d.ply', original_path, frameID);
            draco_fpath = sprintf('%s/%d_draco_cl%d_qp%d.ply', distorted_path, frameID, cl, qp);
            A = pcread(gt_fpath);
            B = pcread(draco_fpath);

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

            fprintf(metrics_file, '%d,%d,%d,%.2f,%.2f,%.2f,%.2f\n', frameID, cl, qp, pssim_geomAB, pssim_geomBA, pssim_colorAB, pssim_colorBA);
        end
    end
end