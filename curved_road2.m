function [pr,pl,estDiff ] = curved_road2(edge_map,est_left,est_right)


 
est_left = round((est_left+200)/2);
est_right = round((est_right+200+360)/2);

estDiff = abs(est_left- est_right);
 
%  est_left =190;
%  est_right = 450;
 
hist_mean_right = zeros(32,2);
hist_mean_left = zeros(32,2);

%Compute the histograms of the small patches
intial_witdh = 160;
stepSize_ver = 10;
stepSize_hor = 4;

for i = 1:23
    
    x_right = max(est_right-intial_witdh/2+i*stepSize_hor/2+1,1):min(est_right+intial_witdh/2-i*stepSize_hor/2,size(edge_map,2));
    x_left = max(est_left-intial_witdh/2+i*stepSize_hor/2+1,1):min(est_left+intial_witdh/2-i*stepSize_hor/2,size(edge_map,2));

    small_edge_map_right = edge_map(1+(i-1)*stepSize_ver:i*stepSize_ver, x_right);
    small_edge_map_left= edge_map(1+(i-1)*stepSize_ver:i*stepSize_ver, x_left);
    
    hist_right = sum(small_edge_map_right);
    hist_left = sum(small_edge_map_left);
    
    hist_mean_right(i,1) = sum(hist_right.*x_right)/sum(hist_right);
    hist_mean_right(i,2) = 10*(i-1);
    
    hist_mean_left(i,1) = sum(hist_left.*x_left)/sum(hist_left);
    hist_mean_left(i,2) = 10*(i-1);
    
end

for i = 24:32
    
    small_edge_map_right = edge_map(1+(i-1)*stepSize_ver:i*stepSize_ver, x_right);
    hist_right = sum(small_edge_map_right);
    
    small_edge_map_left = edge_map(1+(i-1)*stepSize_ver:i*stepSize_ver, x_left);
    hist_left = sum(small_edge_map_left);
    
    hist_mean_right(i,1) = sum(hist_right.*x_right)/sum(hist_right);
    hist_mean_right(i,2) = 10*(i-1);
    
    hist_mean_left(i,1) = sum(hist_left.*x_left)/sum(hist_left);
    hist_mean_left(i,2) = 10*(i-1);
end



%Get rid of NaN's
hist_mean_right = hist_mean_right(find(~isnan(hist_mean_right(:,1))),:);
hist_mean_left = hist_mean_left(find(~isnan(hist_mean_left(:,1))),:);


% figure();
% imshow(edge_map); hold on;
% scatter(hist_mean_right(:,1),hist_mean_right(:,2));


% [minimumX, maximumX, p] = ransaclanes2(hist_mean, ...
%     [input_right*ones(1,1), size(upper_map,1)]);

%% Ransac
[minXr, maxXr, pr] = ransaclanes2(fliplr(hist_mean_right),[]);
[minXl, maxXl, pl] = ransaclanes2(hist_mean_left,[]);


end
