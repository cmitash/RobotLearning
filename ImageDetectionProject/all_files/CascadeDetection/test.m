clc;
clear;
close all;

out = fopen('out.csv','w');
numImages = 1;
%thres = 20;
% thres = [7 8 30 20 30 30 20 25 20 20 15 15];
thres = [0 0 0 0 0 0 0 0 0 0 0 0];
load('features1.mat');
load('features2.mat');
load('features3.mat');
load('features4.mat');
load('features5.mat');
load('features6.mat');
load('features7.mat');
load('features8.mat');
load('features9.mat');
load('features10.mat');
load('features11.mat');
load('features12.mat');

fprintf(out,'%s,%s\n','Id','Category');
for k = 1:numImages
   k
   sol = zeros(1,12);
   img = single(rgb2gray(imread(['test1/image_' num2str(k-1) '.png'])));
    try
        fname = [ 'test1/boxes_' num2str(k-1) '.txt'];
        fid = fopen(fname);
        boxes = textscan(fid,'%s');
    catch err
        
    end
    [rows,columns] = size(boxes{1,1});
    [vf,vd] = vl_sift(img);
    
    %[matches, scores] = vl_ubcmatch(features1,vd,15) ;
    [matches, scores] = vl_ubcmatch(vd,features1,15) ;
    if(size(matches,2) > thres(1))
        sol(1) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features2,5) ;
    if(size(matches,2) > thres(2))
        sol(2) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features3,15) ;
    if(size(matches,2) > thres(3))
        sol(3) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features4,15) ;
    if(size(matches,2) > thres(4))
        sol(4) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features5,15) ;
    if(size(matches,2) > thres(5))
        sol(5) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features6,15) ;
    if(size(matches,2) > thres(6))
        sol(6) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features7,15) ;
    if(size(matches,2) > thres(7))
        sol(7) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features8,15) ;
    if(size(matches,2) > thres(8))
        sol(8) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features9,15) ;
    if(size(matches,2) > thres(9))
        sol(9) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features10,15) ;
    if(size(matches,2) > thres(10))
        sol(10) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features11,15) ;
    if(size(matches,2) > thres(11))
        sol(11) = 1;
    end
    
    [matches, scores] = vl_ubcmatch(vd,features12,15) ;
    if(size(matches,2) > thres(12))
        sol(12) = 1;
    end

    for m=1:rows
        str = textscan(boxes{1,1}{m,1},'%s %d','Delimiter',',');
        fprintf(out,'%s,%d\n',str{1,1}{1,1},sol(m));
    end
end