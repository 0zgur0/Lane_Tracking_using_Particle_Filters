function likelihood = measurementLikelihoodFcn3(PF,predictParticles,measurement_right,measurement_left,varargin)

likelihood = ones(PF.NumParticles,1);
N1 = length(measurement_right);
N2 = length(measurement_left);

for i=1:PF.NumParticles
    particle_indexL = round(predictParticles(i,1));
    particle_indexL = min(max(particle_indexL,1)+200,N2);
    
    particle_indexR = round(predictParticles(i,2));
    particle_indexR = min(max(particle_indexR,1),N1);
    
    likelihood(i,1) = measurement_right(particle_indexR)+measurement_left(particle_indexL);
    %likelihood(i,1) = measurement_left(particle_indexL+200);
    %likelihood(i,1) = measurement_right(particle_indexR);
end


end