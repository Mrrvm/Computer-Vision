%remove background fom imgseq1 and imgseq2
dir_rgb1 = dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/rgb_image1_*.png');
dir_depth1 = dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/depth1_*.mat');

ims=[];
imsd=[];
for i=1:numel(dir_rgb1)
    im=rgb2gray(imread(dir_rgb1(i).name));
    imshow(im); colormap(gray);
    load(dir_depth1(i).name);
    drawnow;
    ims=[ims im(:)];
    imsd=[imsd depth_array(:)];
end

medim=median(double(ims),2);
meddep=median(double(imsd),2);
bgim=(uint8(reshape(medim,[480 640])));
bgimd=reshape(meddep,[480 640]);

for i=1:numel(dir_rgb1)
    im=rgb2gray(imread(dir_rgb1(i).name));
    imshow(im);
    foreg=abs(double(im)-double(bgim))>40;
    figure(3);
    imshow(255*foreg);

    if(i>1) 
        Ia = foreg;
        Ib = foreg_ant;
        % fa,fb is the location of the keypoints
        % (x,y,scale,orientation) 
        % da,db is the descriptor
        % (characterizes the appearence of the keypoint)
        [fa, da] = vl_sift(im2single(Ia));
        [fb, db] = vl_sift(im2single(Ib));
        [matches, scores] = vl_ubcmatch(da, db);
        % this just sorts the matches according 
        %to the scores of each  match
        [drop, perm] = sort(scores, 'descend') ;
        matches = matches(:, perm) ;
        scores  = scores(perm) ;

        % fa(1, :) gives all the x keypoints of image A
        % matches(1, :) gives the columns of fa that were matches in A
        % fa(1, matches(1, :)) gives all the x keypoints matches of A
        xa = fa(1,matches(1,:)) ;
        % size(Ia,2) is just so the points appear in images alongside 
        xb = fb(1,matches(2,:)) + size(Ia,2) ;
        ya = fa(2,matches(1,:)) ;
        yb = fb(2,matches(2,:)) ;

        % this just draws!
        figure(1) ; clf ;
        imagesc(cat(2, Ia, Ib)) ;
        axis image off ;
        figure(2) ; clf ;
        imagesc(cat(2, Ia, Ib)) ;
        hold on ;
        h = line([xa ; xb], [ya ; yb]) ;
        set(h,'linewidth', 1, 'color', 'b') ;
        vl_plotframe(fa(:,matches(1,:))) ;
        fb(1,:) = fb(1,:) + size(Ia,2) ;
        vl_plotframe(fb(:,matches(2,:))) ;
        axis image off ;
        
        pause(2);
    end
    foreg_ant = foreg;
    figure(4);
    imshow(255*foreg_ant);
    
    pause(1);
    
end
    
% get keypoints of one image and show them
%f = vl_sift(im2single(foreg));
%perm = randperm(size(f,2)) ;
%sel = perm(1:83) ;
%h1 = vl_plotframe(f(:,sel)) ;
%h2 = vl_plotframe(f(:,sel)) ;
%set(h1,'color','k','linewidth',3) ;
%set(h2,'color','y','linewidth',2) ;
