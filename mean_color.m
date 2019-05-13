 close all;  clear all;
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
%%
%  vid_circle1 = VideoWriter('test1_circle.mp4','MPEG-4');
% % vid_circle2 = VideoWriter('testbw_circle.mp4','MPEG-4');
%  vid_circle1.FrameRate = vidobj.FrameRate;
% % vid_circle2.FrameRate = vidobj.FrameRate;
%  open(vid_circle1);
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
%      writeVideo(vid_circle1,frame_circle);        % video with circle
%     writeVideo(vid_circle2,bwframe_circle);        % bwvideo with circle
    %d(i)=centroids(1).^2+centroids(2)^2;
%     if i==130
%         tic_x=centroids(1);
%         tic_y=centroids(2);
%     end
    
%     tic_x=[tic_x;centroids(1)];
%     tic_y=[tic_y;centroids(2)];
%     hold on
%    plot(tic_x,tic_y,'LineWidth',2,'Color','r');

end
% i=1:71;
% figure,plot(i,d);
% 
%  close(vid_circle1);
% close(vid_circle2);
%% Read video
figure;title('Comparsion Between Color-morph & Mean Shift Algorithms')
vidobj = VideoReader('test1.MOV');
num_frame = get(vidobj,'NumberOfFrames');
% vid_mean2 = VideoWriter('test1_meanshift.mp4','MPEG-4');
% vid_mean2.FrameRate = vidobj.FrameRate;
%open(vid_mean2);

% for i=9: 240
%     im = read(vidobj,i);
%     filename=['/Users/lylalyla/Documents/19 Spring DIP/Final Project/EE368_FinalProject/EE368_Project_Code/frames/p',num2str(i),'.tif'];
%     imwrite(im,filename,'tif');
% end

%% Crop image for interested object 
I = read(vidobj,130);
%I = read(vidobj,20);

figure(1),imshow(I); title('Original Frame');
[temp,rect]=imcrop(I);
figure,imshow(temp);title('Interested Area');
[s1,s2,s3]=size(temp); 		

%% Decide the weight in the object matrix

y(1)=s1/2;
y(2)=s2/2;                   % Center of the cropped image
tic_x=rect(1)+rect(3)/2;
tic_y=rect(2)+rect(4)/2;
m_wei=zeros(s1,s2);          % Weight based on distance to center
h=y(1)^2+y(2)^2 ;           

for i=1:s1
    for j=1:s2
        dist=(i-y(1))^2+(j-y(2))^2;
        m_wei(i,j)=1-dist/h; %epanechnikov profile
    end
end
C=1/sum(sum(m_wei));         % normalizing coefficient

%% Calculate the weighted histogram

%hist1=C*wei_hist(temp,m_wei,a,b);%target model
%gl=1024;
hist1=zeros(1,4096);     % histogram with 4096 gray levels
%hist1=zeros(1,1024);    % histogram with 1024 gray levels


for i=1:s1
    for j=1:s2
        %rgb each with 16 graylevel s16*16*16=256
        % give each of rgb 16 gral levels
        q_r=fix(double(temp(i,j,1))/16);  
        q_g=fix(double(temp(i,j,2))/16);
        q_b=fix(double(temp(i,j,3))/16);
        q_temp=q_r*256+q_g*16+q_b;            %weight of rgb, k1*k2*k3<4096
        hist1(q_temp+1)= hist1(q_temp+1)+m_wei(i,j);    
    end
end
hist1=hist1*C;
rect(3)=ceil(rect(3));
rect(4)=ceil(rect(4));

%% Video processing

%myfile=dir('/Users/lylalyla/Documents/19 Spring DIP/Final Project/EE368_FinalProject/EE368_Project_Code/frames\*.tif');
%lengthfile=length(myfile);
figure;title('Comparison Between Color-morph and Mean Shift Tracking')
t=zeros(72,2);
for k=130:200
       frame = read(vidobj,k);
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
%      writeVideo(vid_circle1,frame_circle);        % video with circle
%     writeVideo(vid_circle2,bwframe_circle);        % bwvideo with circle
    %d(i)=centroids(1).^2+centroids(2)^2;
%     if i==130
%         tic_x=centroids(1);
%         tic_y=centroids(2);
%     end
    
%     tic_x=[tic_x;centroids(1)];
%     tic_y=[tic_y;centroids(2)];
%     hold on
 %for k=20:100   
    frame = read(vidobj,k);
%     %deblurring Weinner
%     LEN = 41;
%     THETA = 10;
%     noise_var = 40;
%     PSF = fspecial('motion', LEN, THETA);
%     estimated_nsr = noise_var / var(double(frame(:)));
%    
%     wnr2 = deconvwnr(frame, PSF, estimated_nsr);    % deblurred frame
%     %figure, imshow(wnr2)
    % deblurring using Lucy
%     LEN = 36;
%     THETA = 10;
%     PSF = fspecial('motion', LEN, THETA);
%     luc1 = deconvlucy(frame,PSF,5);
    %figure;
    %imshow(luc1);
    
    num=0;
    Y=[2,2];   %step length   
    
    %%%%%%%mean shift iteration conditions
    
    while((Y(1)^2+Y(2)^2>0.5)&num<30)       % condition of iteration
        num=num+1;                          % iteration time
        temp1=imcrop(frame,rect);           % crop the same area in each frame
        %hist2=C*wei_hist(temp1,m_wei,a,b); % target candidates pu
        hist2=zeros(1,4096);
        for i=1:s1
            for j=1:s2
                q_r=fix(double(temp1(i,j,1))/16);
                q_g=fix(double(temp1(i,j,2))/16);
                q_b=fix(double(temp1(i,j,3))/16);
                q_temp1(i,j)=q_r*256+q_g*16+q_b;         % weighted rgb level of each pixel
                % q_temp1(i,j)=q_r*256+q_g*16+q_b;       % weight of rgb for each pixel
                hist2(q_temp1(i,j)+1)= hist2(q_temp1(i,j)+1)+m_wei(i,j);
                % weighted histogram
            end
        end
        hist2=hist2*C;      % normalized with hist to the same scale
        
        %figure(2),subplot(1,2,1),plot(hist2);
        %title('Histogram of Interested Area');hold on;
        
        w=zeros(1,4096);
        for i=1:4096
            if(hist2(i)~=0) 
                w(i)=sqrt(hist1(i)/hist2(i));      % Change of percentage of each gray level value
            else
                w(i)=0;
            end
        end
        
        sum_w=0;    % Comparison of hist1 and hist2
        xw=[0,0];
        for i=1:s1
            for j=1:s2
                sum_w=sum_w+w(uint32(q_temp1(i,j))+1);
                % difference of the gray level value of pixel(i,j)
                % sum_w: the whole difference of the whole histogram
                xw=xw+w(uint32(q_temp1(i,j))+1)*[i-y(1)-0.5,j-y(2)-0.5];
                
            end
        end
        Y=xw/sum_w;
        Y(isnan(Y))=0;
 
        rect(1)=rect(1)+Y(2);
        rect(2)=rect(2)+Y(1);
    end
    
    % Result Show
    tic_x=[tic_x;rect(1)+rect(3)/2];
    tic_y=[tic_y;rect(2)+rect(4)/2];

    % result with rectangle
    v1=rect(1);
    v2=rect(2);
    v3=rect(3);
    v4=rect(4);
    
    c1 = rect(1)+rect(3)/2;
    c2 = rect(2)+rect(4)/2; % center point of the area
    %frame_dot = insertShape(frame,'Line',[tic_x,tic_y-1,tic_x,tic_y+1],'LineWidth',10,'Color','red');
    frame_dot = insertShape(frame,'circle',[c1 c2 0.5*min(s1,s2)],'LineWidth',8);
    
    
    pic=cat(1,frame_circle,frame_dot);
    imshow(pic)
    %imshow(frame_circle);

    %plot(tic_x,tic_y,'LineWidth',2,'Color','r');

    
    %subplot(1,2,2);
    %imshow(uint8(frame_dot));
    
    %imshow(frame_circle);
    %title('Tracking Results');
    
    hold on;
    %plot([v1,v1+v3],[v2,v2],[v1,v1],[v2,v2+v4],[v1,v1+v3],[v2+v4,v2+v4],[v1+v3,v1+v3],[v2,v2+v4],'LineWidth',2,'Color','r');
    %plot([v1,v1+v3],[v2,v2],[v1,v1],[v2,v2+v4],[v1,v1+v3],[v2+v4,v2+v4],[v1+v3,v1+v3],[v2,v2+v4],'LineWidth',2,'Color','yellow');    
    %plot(tic_x,tic_y,'LineWidth',2,'Color','r');
    
    %writeVideo(vid_mean2,frame_dot);        % video with circle
    
end

%close(vid_mean2);