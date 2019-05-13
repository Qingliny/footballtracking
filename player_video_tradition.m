cc
% read the file
vidobj = VideoReader('test1.mp4');
numFrames = get(vidobj,'NumberOfFrames');
disc_locations = zeros(720,1280,numFrames);
% vid_rec1 = VideoWriter('field.mp4','MPEG-4');
vid_rec1 = VideoWriter('player.mp4','MPEG-4');
%%
SE1= strel('disk',5);
% SE2= strel('disk',10);
% SE1= strel('disk',10);
open(vid_rec1);
% open(vid_rec2);
for t =1:numFrames
    rgbImage = read(vidobj,t);
    % red = double(rgbImage(:,:,1));
    % blue = double(rgbImage(:,:,3));
    % red_g = rescale(red);
    % blue = double(rgbImage(:,:,3));
    deal_image=rgbImage;
    for i =1:720
        for j=1:1280
            if deal_image(i,j,1)>200
                deal_image(i,j,1) = 255;
                deal_image(i,j,3) = 0;
            end
            if deal_image(i,j,3)>200 && deal_image(i,j,1) < 100
                deal_image(i,j,3) = 255;
                deal_image(i,j,1) = 0;
            end
        end
    end
    red = double(deal_image(:,:,1));
    red_g = rescale(red);
    BW_red = imbinarize(red_g, 0.5);
    % figure
    % imshow(BW_red);title('BW_red');
    %%
    blue = double(deal_image(:,:,3));
    blue_g = rescale(blue);
    BW_blue = imbinarize(blue_g, 0.5);
%     imshow(BW_blue);title('BW_blue');
%%
    m_open1=imopen(BW_red,SE1);

%     imshow(m_open1);title('red');
    % m_open2= imerode(BW_blue,SE1);
    m_open2=imdilate(BW_blue,SE1);

%     imshow(m_open2);title('blue');
    %%
    % red
    measurements = regionprops(m_open1,'BoundingBox');
%     figure
%     imshow(rgbImage)
%     hold on
    for k = 1 : length(measurements)
        thisBB = measurements(k).BoundingBox;
        if (thisBB(3)/thisBB(4)<1) && (thisBB(3)< 70) &&(thisBB(3)> 20) &&(thisBB(4)<100) &&( thisBB(4)>20)
            pos = [thisBB(1),thisBB(2),thisBB(3),thisBB(4)];
            frame_red = insertShape(rgbImage,'Rectangle',pos,'Color','red','LineWidth',2 );
            writeVideo(vid_rec1,frame_red);  
        end
    end
    % blue
    measurements = regionprops(m_open2,'BoundingBox');
%     hold on
    for k = 1 : length(measurements)
        thisBB = measurements(k).BoundingBox;
        if (thisBB(3)/thisBB(4)<1) && (thisBB(3)/thisBB(4)>0.2) && (thisBB(3)< 70) &&(thisBB(3)> 20) &&(thisBB(4)<100) &&( thisBB(4)>20)
              pos = [thisBB(1),thisBB(2),thisBB(3),thisBB(4)];
              frame_blue = insertShape(rgbImage,'Rectangle',pos,'Color','blue','LineWidth',2 );
              writeVideo(vid_rec1,frame_blue);           % video with rect
        end
    end
end
close(vid_rec1);
% close(vid_rec2);

