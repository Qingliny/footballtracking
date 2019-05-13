%clear, close all
vidobj = VideoReader('test2.MOV');
num_frame = get(vidobj,'NumberOfFrames');       % number of frames
rgbImage1 = read(vidobj,1);                     % first frame of the video
th1 = 0.4;                                        % for blue channel
th2 = 0.6;                                        % for binary image
SE1 = strel('disk',6);                           % SE for opening 
SE2 = strel('disk',13);  
s1 = 1080;
s2 = 1920;
% 
  vid_circle1 = VideoWriter('test1_circle.mp4','MPEG-4');
% % vid_circle2 = VideoWriter('testbw_circle.mp4','MPEG-4');
  vid_circle1.FrameRate = vidobj.FrameRate;
% % vid_circle2.FrameRate = vidobj.FrameRate;
  open(vid_circle1);
% % open(vid_circle2);

d=zeros(200,1);

for i = 130:200
    frame = read(vidobj,i);
    red = double(frame(:,:,1));                 % red channel
    red = rescale(red);
    blue = frame(:,:,3);                        % blue channel
    %hist=histogram(blue);
    %figure,imshow(hist);
    bw = 1-im2bw(blue,th1);                     % mask to elliminate shirt
    %figure,imshow(bw);
    bw_red = (bw).*red;
    %figure,imshow(bw_red);
    bw_final = im2bw(bw_red,th2);               % thresholdin
    
    m1 = imerode(bw_final,SE1);                 % opening 
    m2 = imdilate(m1,SE1);
    m3 = imdilate(m2,SE2);                      % closing
    m4 = imerode(m3,SE2);
    %final=imfill(di,'holes');                  % holes filling
    %figure,imshow(m4);
    m5 = bwareafilt(m4,1); % biggest connected area
    stats = regionprops('table',m5,'Centroid',...
        'MajorAxisLength','MinorAxisLength');   % center of region
    hold on;
    centroids = cat(1,stats.Centroid);          % center pixel
    diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
    radii = diameters/2;                        % radius of area
    %plot(centroids(:,1),centroids(:,2),'b*');  % draw center
    bwframe_circle = insertShape(double(zeros(s1,s2)),'Filledcircle',[centroids radii],'Color','white','Opacity',1.0,'LineWidth',6);
    %figure;imshow(bwframe_circle)
    frame_circle = insertShape(frame,'circle',[centroids radii],'LineWidth',8);
    imshow(frame_circle);
      writeVideo(vid_circle1,frame_circle);        % video with circle
%     writeVideo(vid_circle2,bwframe_circle);        % bwvideo with circle
    d(i)=centroids(1).^2+centroids(2)^2;
%     if i==130
%         tic_x=centroids(1);
%         tic_y=centroids(2);
%     end
    
     tic_x=[tic_x;centroids(1)];
     tic_y=[tic_y;centroids(2)];
     hold on
    plot(tic_x,tic_y,'LineWidth',2,'Color','r');

end
% i=1:71;
% figure,plot(i,d);
% 
  close(vid_circle1);
% close(vid_circle2);