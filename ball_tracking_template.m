clear all
close all
%% Prepare the image for analysis
vidobj = VideoReader('test1.MOV');
num_frame = get(vidobj,'NumberOfFrames');
vid_rec1 = VideoWriter('ball_tracking_test1.mp4','MPEG-4');
vid_rec1.FrameRate = vidobj.FrameRate;

%%
% vidobj = VideoReader('test2.MOV');
% I = rgb2gray(read(vidobj,200));
% %I = read(vidobj,20)
% figure(1),imshow(I);
% [temp,rect]=imcrop(I);
% filename='template_test1_7.tif';
% imwrite(temp,filename,'tif');
%%
open(vid_rec1);
dist = zeros(313,1);
for p = 130:200
    p
    I = read(vidobj,p); % read in coins image
    F = rgb2gray(I); % read in template image
%% correlation matching
    last_max = 0;
    for t = 1:7
        formatSpec = 'template_test1_%d.tif';
        str = sprintf(formatSpec,t);
        T= imread(str);
        [corrScore, boundingBox_time,maxVal] = corrMatching(F,T);
        if maxVal > last_max
            last_max = maxVal;
            boundingBox = boundingBox_time;
        end
    end
%     if p ==1
%         last_location = boundingBox;
%     end
%     if abs(boundingBox(2) - last_location(2)) > 300  || abs(boundingBox(1) - last_location(1)) > 500
%         continue
%     else
    frame = I;
% 
%     dist(p) = sqrt((boundingBox(1)-last_location(1)).^2+(boundingBox(2)-last_location(2)).^2);
%     
%     if boundingBox(1,:) ~= [0,0,0,0]
%         last_location = boundingBox;
%     end

%     figure
%     imshow(I),title('Ball Tracking'),set(gca,'fontsize',20,'fontname','Times');
%     hold on
%     rectangle('Position', [boundingBox(2),boundingBox(1),boundingBox(4),boundingBox(3)],'EdgeColor','yellow','LineWidth',5 )
    pos = [boundingBox(2),boundingBox(1),boundingBox(4),boundingBox(3)];
    frame = insertShape(frame,'Rectangle',pos,'Color','yellow','LineWidth',5 );
%     end
    writeVideo(vid_rec1,frame);
%% show results
% % 
%     figure
%     imshow(F)
%     hold on
%     rectangle('Position', [boundingBox(2),boundingBox(1),boundingBox(4),boundingBox(3)],'EdgeColor','b','LineWidth',2 )
end
close(vid_rec1);
