
% bitrate_split_dynamic('160906_band2_with_ground', 0, 5500, 1, 60000)
% bitrate_split_dynamic('170307_dance5_with_ground', 0, 10100, 0, 60000)
% bitrate_split_dynamic('170915_office1_with_ground', 0, 5900, 2, 60000)
% bitrate_split_dynamic('160906_band2_with_ground', 0, 5500, 1, 80000)
% bitrate_split_dynamic('160906_band2_with_ground', 0, 5500, 1, 100000)
% bitrate_split_dynamic('160906_band2_with_ground', 0, 5500, 1, 120000)
% bitrate_split_dynamic('160906_band2_with_ground', 0, 5500, 1, 160000)
% bitrate_split_dynamic('160906_band2_with_ground', 0, 5500, 1, 200000)

function bitrate_split_dynamic(seq_name, start_frame, end_frame, log_id, bitrate_kbps)
    base_path = '/datassd/pipeline_cpp/client_tiled/ablation';
    culling = 's_nocull_c_nocull';
    % culling = 's_cull_c_cull';
    original_path = sprintf('%s/bitrate_study/%s/o3d_gt_with_ground_%s/log%d/', base_path, seq_name, culling, log_id);
    distorted_path = sprintf('%s/bitrate_dynamic_split/%s/o3d_nvenc_%dk_fps30_%s/log%d/', base_path, seq_name, bitrate_kbps, culling, log_id);
    comp_path = sprintf('%s/bitrate_dynamic_split/%s/comp_o3d_pointssim_%s/log%d/', base_path, seq_name, culling, log_id);

    if ~exist(comp_path, 'dir')
        mkdir(comp_path);
    end

    out_file_path = sprintf('%s/3D_%dk_pssim.csv', comp_path, bitrate_kbps);
    metrics_file = fopen(out_file_path, 'w');

    fprintf(metrics_file, 'Frame,PSSIM_GEO_AB,AVG_PSSIM_GEO_AB,PSSIM_GEO_BA,AVG_PSSIM_GEO_BA,PSSIM_COLOR_AB,AVG_PSSIM_COLOR_AB,PSSIM_COLOR_BA,AVG_PSSIM_COLOR_BA\n');

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
    for frameID = start_frame:200:end_frame

        %% Load point clouds
        % gt_fname = sprintf("%s%d.ply", original_path, frameID)
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
end