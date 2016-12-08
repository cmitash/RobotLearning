clc;
clear;
close all;

out = fopen('out.csv','w');
fprintf(out,'%s,%s,%s,%s,%s\n','Id','bb_tl_x','bb_tl_y','bb_lr_x','bb_lr_y');
testImages = 127;

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

for k = 1:testImages
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
    
    [matches, scores] = vl_ubcmatch(vd,features1,5) ;
    size(matches,2)
        
    
    [matches, scores] = vl_ubcmatch(vd,features2,5) ;
    
    
    [matches, scores] = vl_ubcmatch(vd,features3,5) ;
    
    
    [matches, scores] = vl_ubcmatch(vd,features4,5) ;
    
    
    [matches, scores] = vl_ubcmatch(vd,features5,5) ;
    
    
    [matches, scores] = vl_ubcmatch(vd,features6,5) ;
    
    
    [matches, scores] = vl_ubcmatch(vd,features7,5) ;
    
    
    [matches, scores] = vl_ubcmatch(vd,features8,5) ;
    
    
    [matches, scores] = vl_ubcmatch(vd,features9,5) ;
   
    
    [matches, scores] = vl_ubcmatch(vd,features10,5) ;
    
    
    [matches, scores] = vl_ubcmatch(vd,features11,5) ;
    
    
    [matches, scores] = vl_ubcmatch(vd,features12,5) ;
    

    for m=1:rows
        str = textscan(boxes{1,1}{m,1},'%s %d','Delimiter',',');
        fprintf(out,'%s,%d\n',str{1,1}{1,1},sol(m));
    end
end