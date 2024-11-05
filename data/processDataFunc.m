function [source_points, source_rms, source_rms_mean, source_variance] = processDataFunc(source_data, data_N, map_data, cones_N)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    source_N = source_data(1:2:end,1);
    source_points = zeros(2,data_N);
    source_variance = zeros(2, data_N);
    source_rms = zeros(data_N,1);

    for i = 1:data_N
        if source_N(i) == 0
            source_rms(i) = 0;
            source_points(:,i) = [0;0];
        else
            source_points(:,i) = median(source_data(2*i-1:2*i, 2:source_N(i)+1),2);
            
            dist_map = zeros(cones_N,1);
            for j = 1:cones_N
                dist_map(j) = norm(source_points(:,i) - map_data(:,j));
            end
            [~, min_dist_map_idx] = min(dist_map);
    
            x = zeros(1,source_N(i));
            for k = 1:source_N(i)
                x(k) = norm(source_data(2*i-1:2*i,k+1) - map_data(:,min_dist_map_idx));
            end
            source_rms(i) = rms(x);
            source_variance(:, i) = (sum((source_data(2*i-1:2*i, k+1:source_N(i)+1) - map_data(:,min_dist_map_idx)).^2,2) / (source_N(i) - 1))';
            
        end
    end

source_rms_mean = sum(source_rms(source_rms > 0)' * source_N(source_N > 0)) / sum(source_N);
%source_rms_mean = median(source_rms(source_rms > 0));
source_points = source_points(:,source_N > 0);
source_variance = source_variance(:, source_N > 0);

end