function []= animation(System)
 
 %% 1. define the motion coordinates 
 
    tt    = 1:numel(System.t(:));
    z     = System.state(2,:);
    y     = System.state(1,:);
    x     = y*0;
    yaw   = x ;
    roll  = System.state(3,:)*180/pi ;
    pitch = x;


 %% 6. animate by using the function makehgtform
 % Function for ANimation of QuadCopter
 [movieVector]= drone_Animation(x,y,z,roll,pitch,yaw)
 
 
 %% step5: Save the movie
% myWriter = VideoWriter('drone_animation', 'Motion JPEG AVI');
myWriter = VideoWriter('drone_animation1', 'MPEG-4');
myWriter.Quality = 100;
myWritter.FrameRate = 120;
myWritter.CompressionRatio=1;
% Open the VideoWriter object, write the movie, and class the file
open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter); 