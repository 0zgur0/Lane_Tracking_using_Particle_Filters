
outputVideo = VideoWriter('curved\curved.avi');
outputVideo.FrameRate = 25;
open(outputVideo)

workingDir = 'curved';
imageNames = dir(workingDir);

for i = 3:3625
   filename = [sprintf('%03d',i) '.jpg'];
   fullname = fullfile('curved/',filename);
   img = imread(fullname);
   writeVideo(outputVideo,img);
end

close(outputVideo);