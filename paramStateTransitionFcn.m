function predictParticles = paramStateTransitionFcn( pf,prevParticles,stateEst,varargin )

sigma1 = 0.001;
sigma2 = 0.05;
sigma3 = 20;


N = pf.NumParticles;

predictParticles = zeros(N,3);

for i=1:N
    predictParticles(i,1) = prevParticles(i,1) + sigma1*randn();
    predictParticles(i,2) = prevParticles(i,2) + sigma2*randn();
    predictParticles(i,3) = prevParticles(i,3) + sigma3*randn();
end


end

