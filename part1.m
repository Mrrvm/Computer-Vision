%function objects = track3D_part1(imgseq1, imgseq2, cam_params, cam1toW, cam2toW)

	%remove background fom imgseq1 and imgseq2
    dir_rgb1 = dir('data_rgb/rgb_image1_*.png');
	dir_rgb2 = dir('data_rgb/rgb_image2_*.png');
	dir_depth1 = dir('data_rgb/depth1_*.mat');
	dir_depth2 = dir('data_rgb/depth2_*.mat');
    
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
            [fa, da] = vl_sift(im2single(Ia));
            [fb, db] = vl_sift(im2single(Ib));
            [matches, scores] = vl_ubcmatch(da, db);
            [drop, perm] = sort(scores, 'descend') ;
            matches = matches(:, perm) ;
            scores  = scores(perm) ;

            figure(1) ; clf ;
            imagesc(cat(2, Ia, Ib)) ;
            axis image off ;

            figure(2) ; clf ;
            imagesc(cat(2, Ia, Ib)) ;

            xa = fa(1,matches(1,:)) ;
            xb = fb(1,matches(2,:)) + size(Ia,2) ;
            ya = fa(2,matches(1,:)) ;
            yb = fb(2,matches(2,:)) ;

            hold on ;
            h = line([xa ; xb], [ya ; yb]) ;
            set(h,'linewidth', 1, 'color', 'b') ;

            vl_plotframe(fa(:,matches(1,:))) ;
            fb(1,:) = fb(1,:) + size(Ia,2) ;
            vl_plotframe(fb(:,matches(2,:))) ;
            axis image off ;
            pause(1);
        end
        foreg_ant = foreg;
        figure(4);
        imshow(255*foreg_ant);
        
        
    end
    

    %f = vl_sift(im2single(foreg));
    %perm = randperm(size(f,2)) ;
	%sel = perm(1:83) ;
	%h1 = vl_plotframe(f(:,sel)) ;
	%h2 = vl_plotframe(f(:,sel)) ;
	%set(h1,'color','k','linewidth',3) ;
	%set(h2,'color','y','linewidth',2) ;

    
%end




