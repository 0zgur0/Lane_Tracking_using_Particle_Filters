clear; %close all;clc;
%Load projective transform matrix
load('tform.mat');
%load('f00123.png.mat');

%Image width 
img_width = 360; 

%SkySize
sky = 0.6*404;

%Image directory
img_directory = 'our_data/';

%Create a particle filter object
 pf = robotics.ParticleFilter;
 pfC = robotics.ParticleFilter;

%Choose resmapling policy
policy = robotics.ResamplingPolicy;
%policy.TriggerMethod = 'interval';
%policy.SamplingInterval = 2;
pf.ResamplingPolicy = policy;
%pfC.ResamplingPolicy = policy;
%pfC.StateEstimationMethod = 'maxweight';

%Choose the likelihood function for particle filter
pf.MeasurementLikelihoodFcn = @measurementLikelihoodFcn3;
pfC.MeasurementLikelihoodFcn = @paramLikelihoodFcn;

%Define the state transition function
pf.StateTransitionFcn = @stateTransitionFcn3;
pfC.StateTransitionFcn = @paramStateTransitionFcn;



%% Initilize the particle filter
 mean = [img_width/4,img_width*(3/4)];
 covariance = img_width/4*eye(2);
 initialize(pf,1000,mean,covariance);

 %Initilize second particle filter-pfC 
 %[0.0007   -0.2586  422.3815];
 initilParam = [0  0  422]';
 initialParamCov = zeros(3);
 initialParamCov(1,1) = 0.001;
 initialParamCov(2,2) = 0.2;
 initialParamCov(3,3) = 10;
 initialize(pfC,1000,initilParam,initialParamCov);

%Define counter for image writing
ii= 1;

%Define state Estimate
stateEst =zeros(1,2);
stateEstC = zeros(1,3);

%Initial and last time frame
iT = 835;
T = 7000;

%Particle filter iteration
for i=iT:T %i=time stamp
    tic,
    %% Prediction - (drift and diffusion)
    %figure();plot(pf_Right.Particles);hold on;
    [statePredicted,stateCov] = predict(pf,pf.Particles,stateEst);
    %plot(pf_Right.Particles,'r');
    
    %% Measurement from current image
    %For now measuremnet is horizontal histogram
    
    %Read the image
    img_name = strcat(img_directory,num2str(i),'.jpg');
    img= imread(img_name);
    
    %Find a histogram for lower image
    [lower_map,edge_map_small,img_crop ,img_tf]  = cues2( img,tform1);
    
    lower_map_right = lower_map(:,size(lower_map,2)/2:end);
    lower_map_left = lower_map(:,1:size(lower_map,2)/2);
    
    %measurement is histogram
    measurement_right = sum(lower_map_right);
    if norm(measurement_right) >0
        measurement_right = measurement_right/norm(measurement_right)+ 2/length(measurement_right);
    else
        measurement_right = (5/length(measurement_right))*ones(1,size(lower_map_right,2));
    end
    
    measurement_left = sum(lower_map_left);
    if norm(measurement_left) >0
        measurement_left = measurement_left/norm(measurement_left)+ 2/length(measurement_left);
    else
        measurement_left = (5/length(measurement_left))*ones(1,size(lower_map_left,2));
    end
    
    %% Prediction - pfC
    [statePredictedC,stateCovC] = predict(pfC,pfC.Particles,stateEstC);
    
    %% Correct weights by using measurement
    [stateCorrected,stateCov] = correct(pf,measurement_right,measurement_left);
    
    
    %% Estimate the state according to selected estimation method (default:mean)
    stateEst = getStateEstimate(pf);
    
    
    %% --------------------------------------------------------------------------------
    
    %% Upper part - could be curved
    % Measurement from RANSAC
    [pr,pl,estDiff] = curved_road2(edge_map_small,stateEst(1),stateEst(2));
    
    %% Correct weights by using measurement
    [stateCorrectedC,stateCovC] = correct(pfC,stateEst,pr,pl);
    
    %% Estimate the state according to selected estimation method (default:mean)
    stateEstC = getStateEstimate(pfC);
    
    %% Consistency Check
%     if ii==1;
%         previousPoly = pr;
%     end
%     [consistent, previousPoly] = consistency_check( pr,pl, previousPoly,stateEst );
%     
%     pr = consistent*pr + (1-consistent)*previousPoly;
    
    %% Generate points on lines
    yr = (-400:10:250)';
    xr = polyval(stateEstC,yr);
    
    yr = 2*yr;
    xr = 2*xr;
    
    %xr = xr - (xr(find(yr == 160))-(stateEst(2)+560));
    yl = yr;
    xl = xr - 2*estDiff;
      
    %Find the point in the real image
    [uR,vR] = tforminv(tform1,xr-200,yr-500);
    [uL,vL] = tforminv(tform1,xl-200,yl-500);
    
    vR = vR+242;
    vL = vL+242;

    uvR = [uR,vR];
    uvL = [uL,vL];


    %% Visulize
    %Find the point in the real image
    [uR,vR] = tforminv(tform1,stateEst(2)+360,160);
    [uL,vL] = tforminv(tform1,stateEst(1),160);
    
    %Extra 2 points for drawing
    [uRE,vRE] = tforminv(tform1,stateEst(2)+360,-10);
    [uLE,vLE] = tforminv(tform1,stateEst(1),-10);
    
    %img_mark = insertShape(img,'FilledCircle',[uL,size(img,1)-10,5],'LineWidth',3, 'Color','blue');
    %img_mark = insertShape(img_mark,'FilledCircle',[uR,size(img,1)-10,5],'LineWidth',3, 'Color','red');
    
    %Draw polygon on the input image
%      polygon_points = [uL,vL+sky,uLE,vLE+sky,uRE,vRE+sky,uR,vR+sky];
%     img_mark = insertShape(img,'FilledPolygon',polygon_points,'LineWidth',1, 'Color','green');
%     
%          %polygon_points = [[uL,vL+sky;uLE,vLE+sky];flipud(uvL); uvR;[uRE,vRE+sky;uR,vR+sky]];
%          polygon_points = [uvL; flipud(uvR)];
%          polygon_points = polygon_points';
%          polygon_points = polygon_points(:);
%          polygon_points = polygon_points';
%          img_mark = insertShape(img_mark,'FilledPolygon',polygon_points,'LineWidth',1, 'Color','red');
    
    %img_mark = insertShape(img_tf,'FilledCircle',[stateEst(1)+200,size(img_tf,1)-20,5],'LineWidth',3,'Color','red');
    %img_mark = insertShape(img_mark,'FilledCircle',[[uvL;uvR],2*ones(size(uvL,1)+size(uvR,1),1)],'LineWidth',1);
    %imshow(img_mark);
    
    %    %Image Writing
%         filename = [sprintf('%03d',ii) '.jpg'];
%         fullname = fullfile('curved2/',filename);
%         imwrite(img_mark,fullname)    % Write out to a JPEG file (img1.jpg, img2.jpg, etc.)
%         ii = ii+1;
  toc  
end
