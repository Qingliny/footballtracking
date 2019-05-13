cc
%%
% read the file
vidobj = VideoReader('test1.mp4');
numFrames = get(vidobj,'NumberOfFrames');
vid_rec1 = VideoWriter('player_kmeans1.mp4','MPEG-4');
vid_rec1.FrameRate = vidobj.FrameRate;
SE1= strel('disk',5);
SE2= strel('disk',10);
%%
% do k-means color segmentation
% open(vid_rec1);
for t = 198
% for t=27
    rgbImage = read(vidobj,t);
    deal_image = rgbImage;
    for i =1:720
        for j=1:1280
            if deal_image(i,j,2)>100
                deal_image(i,j,1) = 255;
                deal_image(i,j,2) = 255;
                deal_image(i,j,3) = 255;
            end
%             if deal_image(i,j,3)>100 && deal_image(i,j,1) <100
%                 deal_image(i,j,3) = 255;
%                 deal_image(i,j,1) = 0;
%             end
        end
    end
    p11 = deal_image;
    figure
    imshow(deal_image)
%%
    % do k-means on teams
    [P,Centers] = imsegkmeans(deal_image,3);
    if Centers(1,1) == max(Centers(:,1))
        P_back = P==1;
        if Centers(2,1) > Centers(3,1) 
            P_red = P==2;
            P_blue = P==3;
        else
            P_blue = P==2;
            P_red = P==3;
        end
    elseif Centers(2,1) == max(Centers(:,1))
        P_back = P==2;
        if Centers(1,1) > Centers(3,1) 
            P_red = P==1;
            P_blue = P==3;
        else
            P_blue = P==1;
            P_red = P==3;
        end
    elseif Centers(3,1) == max(Centers(:,1))
        P_back = P==3;
        if Centers(1,1) > Centers(2,1) 
            P_red = P==1;
            P_blue = P==2;
        else
            P_blue = P==1;
            P_red = P==2;
        end
    end
%     %%
%     P = imopen(P,SE1);
    B = labeloverlay(deal_image,P);
    p12 = B;
    imshow(B)
    %%
    % red
%     P_red1 = imopen(P_red,SE1);
    P_red1 = imclose(P_red,SE2);
% %     P_red1 = imopen(P_red1,SE1);

    measurements = regionprops(P_red1,'BoundingBox');
    figure
    imshow(rgbImage),title('Original Frame'),set(gca,'fontsize',20,'fontname','Times');
    figure
    imshow(p11),title('Filting the Green Field'),set(gca,'fontsize',20,'fontname','Times');
    figure
    imshow(p12),title('Labeling the region with K-means'),set(gca,'fontsize',20,'fontname','Times');
    figure
    imshow(rgbImage),title('Tracking Players'),set(gca,'fontsize',20,'fontname','Times');
    hold on
    frame_red = rgbImage;
    for k = 1 : length(measurements)
        thisBB = measurements(k).BoundingBox;
        if (thisBB(3)/thisBB(4)<1) && (thisBB(3)>10)&&(thisBB(4)>40)
          rectangle('Position', [thisBB(1),thisBB(2),thisBB(3),thisBB(4)],...
          'EdgeColor','r','LineWidth',2 )
%             pos = [thisBB(1),thisBB(2),thisBB(3),thisBB(4)];
%             frame_red = insertShape(frame_red,'Rectangle',pos,'Color','red','LineWidth',5 );
        end
    end
    % blue
%     hold on 
    P_blue1 = imclose(P_blue,SE2);
%     P_blue1 = imopen(P_blue1,SE1);
%     P_blue1 = imerode(P_blue1,SE2);
    measurements = regionprops(P_blue1,'BoundingBox');
%     hold on
    for k = 1 : length(measurements)
        thisBB = measurements(k).BoundingBox;
        if (thisBB(3)/thisBB(4)<1) && (thisBB(3)>10)&&(thisBB(4)>40)
%             pos = [thisBB(1),thisBB(2),thisBB(3),thisBB(4)];
%             frame_red = insertShape(frame_red,'Rectangle',pos,'Color','blue','LineWidth',5 );
           % video with rect
            rectangle('Position', [thisBB(1),thisBB(2),thisBB(3),thisBB(4)],...
            'EdgeColor','b','LineWidth',2 )
        end
    end

%     writeVideo(vid_rec1,frame_red);
end
% close(vid_rec1);