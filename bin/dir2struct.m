function [x,y,imgseq1,imgseq2]=dir2struct(d_f_d,rgb_f_d)
	[~,Dformat]=strtok(d_f_d,'.');
	[~,RGBformat]=strtok(rgb_f_d,'.');
	D=dir(strcat('*',Dformat));
	RGB=dir(strcat('*',RGBformat));
	[n,~]=size(D);
	sz=(n-1)/2;
	sample=load(D(2).name);
	[x,y]=size(sample.depth_array);
	imgseq1= struct('rgb',cell(sz,1),'depth',cell(sz,1));
	imgseq2= struct('rgb',cell(sz,1),'depth',cell(sz,1));
	for i=2:n
	    set=sscanf(D(i).name,d_f_d);
	    if set(1) == 1
	        imgseq1(set(2)).rgb = RGB(i-1).name;
	        imgseq1(set(2)).depth = D(i).name;
	    else
	        imgseq2(set(2)).rgb = RGB(i-1).name;
	        imgseq2(set(2)).depth = D(i).name;
	    end
	end
end
