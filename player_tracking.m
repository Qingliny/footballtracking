clear, close all
vidobj = VideoReader('IMG_4776.MOV');
num_frame = get(vidobj,'NumberOfFrames');
 vid_rec1 = VideoWriter('player_rec.mp4','MPEG-4');
% vid_rec2 = VideoWriter('playerbw_rec.mp4','MPEG-4');
 vid_rec1.FrameRate = vidobj.FrameRate;
% vid_rec2.FrameRate = vidobj.FrameRate;
 open(vid_rec1);
% open(vid_rec2);

for i = 130:200
    rgbImage = read(vidobj,i);
    hsv = rgb2hsv(rgbImage);                    % HSV colorspace
    H = hsv(:,:,3);                             % H component
    hsv2 = im2bw(H,0.8);
    I = bwareafilt(hsv2,2);                      % two largest connected areas
    stats = regionprops('table',I,'Centroid',...
        'MajorAxisLength','MinorAxisLength');   % center point
    centers = stats.Centroid;               
    centroids_p1(i,:) = centers(1,:);
    centroids_p2(i,:) = centers(2,:);
    pos_p1 = [centroids_p1(i,1)-75 centroids_p1(i,2)-100 200 300];
    pos_p2 = [centroids_p2(i,1)-75 centroids_p2(i,2)-100 200 300];
    pos = vertcat(pos_p1,pos_p2);
    frame_withrecs = insertShape(double(I),'Rectangle',pos,'Color','yellow','LineWidth',5);
    frame_final = insertShape(rgbImage,'Rectangle',pos,'Color','yellow','LineWidth',5);
    writeVideo(vid_rec1,frame_final);           % video with rect
    %writeVideo(vid_rec2,frame_withrecs);        % bwvideo with rect
end

close(vid_rec1);
%close(vid_rec2);