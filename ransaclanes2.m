function [minimumX, maximumX, p] = ransaclanes2(points,constraintpoints)

global minX maxX
minX = min(points(:,1));
maxX = max(points(:,1));

numSample = round(size(points,1)/2);

% Ransac
[M, inliers] = ransac2(points', constraintpoints', @fittingfn, @distfn, @degenfn, numSample, 10, 0, 100, 2000);

% disp 'Number of inliers'
% size(inliers)

if size(M,2)>1
    p = M;
    minimumX = min(inliers(:,1));
    maximumX = min(inliers(:,2));

    % Plot inliers and outliers
%     %figure(6);
%     x1 = linspace(minX-50,maxX+50);
%     y1 = polyval(M,x1);
%     plot(y1, x1, 'Color', 'blue');
%     hold on
%     plot(points(:,2), points(:,1),'.','Color','red');
%     plot(inliers(:,2), inliers(:,1),'.','Color','green');
% %     legend('inliers', 'outliers');
%     title('RANSAC Result');
%     hold off

else
    minimumX = 0;
    maximumX = 0;
    p = [];
end

%% Fitting function
function [M] = fittingfn(x)
   M = polyfit(x(1,:),x(2,:),2);
 
   %M = mmpolyfit(x(1,:), x(2,:),2,'Point',constraintx');
   
   

%% Distance function
function [inliers, M] = distfn(M, x, t)
%This function must evaluate the distances between points
%and the model returning the indices of elements in x that
%are inliers, that is, the points that are within distance
%'t' of the model
    M = M;
    x2 = linspace(min(x(1,:)),max(x(1,:)),30);
    y2 = polyval(M,x2);


    X = [x2' y2'];
    XI = x';
    k = dsearchn(X, XI);
    distances = diag(pdist2(XI, X(k,:)));
    inliers = XI(find(distances<t),:);
    outliers = XI(find(distances>=t),:);

    
% Check if degenerate function
function [r] = degenfn(x,M)

    r = 0;





