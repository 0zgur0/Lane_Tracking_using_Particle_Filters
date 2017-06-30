function predictParticles = stateTransitionFcn3(pf,prevParticles,stateEst,varargin)

sigma1 = 10;
sigma2 = 10;

thL = 300;
thR = 40;

N = pf.NumParticles;

predictParticles = zeros(N,2);

witdh = 520;

if stateEst(1) > thL
    for i=1:N
         predictParticles(i,2) = prevParticles(i,1) + sigma2*randn() - 360;    
         predictParticles(i,1) = prevParticles(i,1) + sigma1*randn() - witdh;
    end
else
    for i=1:N
          predictParticles(i,1) = prevParticles(i,1) + sigma1*randn();
          predictParticles(i,2) = prevParticles(i,2) + sigma2*randn();
    end

end


end