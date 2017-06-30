function likelihood = paramLikelihoodFcn( PF,predictParticles,stateEst,pr,pl,varargin )

likelihood = ones(PF.NumParticles,1);

est_right = (stateEst(2)+200+360)/2;

% mu1 = pr(1);
% mu2 = pr(2);
 mu3 =  512;
  
sigma3 = 10;
sigma2 = 0.1;
sigma1 = 0.001;

invCv = zeros(3);
invCv(1,1) = 1/sigma1^2;
invCv(2,2) = 1/sigma2^2;
invCv(3,3) = 1/sigma3^2;

for i=1:PF.NumParticles 
 res = (predictParticles(i,:)-pr)';
 likelihood(i,1) = exp(-0.5*res'*(invCv*res));
end

% likelihood(:,1) = normpdf(predictParticles(:,3),mu3,sigma3);
end

