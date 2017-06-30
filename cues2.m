function [ lower_map,edge_map_small, img_crop, img_tf] = cues2( img, tform1)

% Crop and transform it
%Crop the upper part of the image
img_crop=imcrop(img,[1, 0.6*size(img,1), size(img,2), 0.4*size(img,1)]);

%Sharpen image
img_crop = imsharpen(img_crop,'Amount',1.8);

%Transform the image - bird-eye view
img_tf = imtransform(img_crop,tform1,'XData',[-199,size(img_crop,2)+200],'YData', [-500,160],'XYScale',1);

%Downscale transformed image
img_tf_small = imresize(img_tf,0.5);

grayim = rgb2gray(img_tf); % 255 shades of Grey

edge_map = zeros(size(grayim,1),size(grayim,2)); % Initialize edge map

%Downscaled edge map for upper part 
edge_map_small = edge(rgb2gray(img_tf_small),'sobel');

% Distance of assessment, depends on image resolution
% Roughly, the bigger the m, more edges are found
m = 20;
% Treshold of value difference
% Quite robust to change in lighting, the bigger the less edges are found
T = 20;  

for i=1:size(edge_map,1)
    for j=m+1:size(edge_map,2)-(m+1) % columns are the horizontal dimension
        
        b_plus = grayim(i,j) - grayim(i,j+m);
        b_minus = grayim(i,j) - grayim(i,j-m);
        
        if (b_plus > 0 && b_minus > 0)
            if (b_plus + b_minus >= T)
                edge_map(i,j) = 1;
            end
        end
    end
end


% Neighborhood deletion
n = 250; % Maximum number of pixels in the neigborhood
edge_map = bwareaopen(edge_map,n);


% Hysteresis reconstuction -----------------> Thin Lines get destroyed
s = 2;  % Size of the structure element
SE = strel('disk', s); % Structure element
seed = imerode(edge_map,SE); % Erode
edge_map = imreconstruct(seed,edge_map); % use Seeds to reconstruct
%figure();imshow(edge_map);
% figure();imshow(img_tf);

%TODO
% %% Color space detection
% 
% %HSI color space, taken from internet
% imhsi = rgbtohsi(im_crop);
% whiteareas = imhsi(:,:,2)<0.03 & imhsi(:,:,3)>0.65;
% % figure(); imshow(whiteareas);
% 
% yellowareas = imhsi(:,:,1)>0.05 & imhsi(:,:,1)<0.17 & imhsi(:,:,2)>0.2 & imhsi(:,:,2)<0.5 & imhsi(:,:,3)>0.5;
% % figure(); imshow(yellowareas);
% 
% mask = yellowareas | whiteareas;
% % figure(); imshow(mask);
% 
% % Results
% if (show_figs == true)
%     figure(); imshow(mask);
%     title('Color Edge map');
% end

%% Final Edge map

%Merge both edge maps
y_half = round(size(edge_map,1)*0.7);
lower_map = edge_map(y_half:end,:);


end

