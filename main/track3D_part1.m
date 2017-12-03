function objects = track3D_part1(imgseq1, imgseq2, cam_params, cam1toW, cam2toW)

    ims1=[]; ims2=[];
    imsd1=[]; imsd2=[];
    for i=1:numel(imgseq1)
        im1=rgb2gray(imread(imgseq1(i).rgb));
        im2=rgb2gray(imread(imgseq2(i).rgb));
        %{
        figure(1);
        imshow(im1); colormap(gray);
        figure(2);
        imshow(im2); colormap(gray);
        drawnow;
        %}    
        load(imgseq1(i).depth);
        imsd1=[imsd1 depth_array(:)];
        clear depth_array;
        load(imgseq2(i).depth);
        imsd2=[imsd2 depth_array(:)];
        clear depth_array;
        ims1=[ims1 im1(:)];
        ims2=[ims2 im2(:)];
    end
    
    % calculates background image for cam1 and 2
    medim1=median(double(ims1),2);
    medim2=median(double(ims2),2);
    meddep1=median(double(imsd1),2);
    meddep2=median(double(imsd2),2);
    bgim1=(uint8(reshape(medim1,[480 640])));
    bgim2=(uint8(reshape(medim2,[480 640])));
    bgimd1=reshape(meddep1,[480 640]);
    bgimd2=reshape(meddep2,[480 640]);

    % calculates foreground per frame for cam1 and cam 2
    for i=1:numel(imgseq1)
        im1=rgb2gray(imread(imgseq1(i).rgb));
        foreg1=bwareafilt(abs(double(im1)-double(bgim1))>40,[1000 480*640]);
        foreg1 =bwlabel(foreg1);
        im2=rgb2gray(imread(imgseq2(i).rgb));
        foreg2=bwareafilt(abs(double(im2)-double(bgim2))>40,[1000 480*640]);
        foreg2 =bwlabel(foreg2);
        %{
        figure(3);
        imshow(255*foreg1);
        figure(4);
        imshow(255*foreg2);
        pause(1);
        %}
    end

    objects = 1;
    
end




