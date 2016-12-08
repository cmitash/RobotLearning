clc;
clear ;
close all;

out = fopen('out.csv','w');

%Load the features extracted for each object
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

%Define detectors trained for each object
detector1 = vision.CascadeObjectDetector('obj1.xml','MinSize',[100 100],'MaxSize',[200 200]);
detector2 = vision.CascadeObjectDetector('obj2.xml','MinSize',[100 100],'MaxSize',[200 200]);
detector3 = vision.CascadeObjectDetector('obj3.xml','MinSize',[100 100],'MaxSize',[250 200]);
detector4 = vision.CascadeObjectDetector('obj4.xml','MinSize',[100 100],'MaxSize',[200 200]);
detector5 = vision.CascadeObjectDetector('obj5.xml','MinSize',[100 100],'MaxSize',[200 200]);
detector6 = vision.CascadeObjectDetector('obj6.xml','MinSize',[100 100],'MaxSize',[200 200]);
detector7 = vision.CascadeObjectDetector('obj7.xml','MinSize',[100 100],'MaxSize',[200 200]);
detector8 = vision.CascadeObjectDetector('obj8.xml','MinSize',[100 100],'MaxSize',[200 250]);
detector9 = vision.CascadeObjectDetector('obj9.xml','MinSize',[100 100],'MaxSize',[200 200]);
detector10 = vision.CascadeObjectDetector('obj10.xml','MinSize',[100 100],'MaxSize',[200 200]);
detector11 = vision.CascadeObjectDetector('obj11.xml','MinSize',[100 100],'MaxSize',[200 200]);
detector12 = vision.CascadeObjectDetector('obj12.xml','MinSize',[100 100],'MaxSize',[200 200]);

fprintf(out,'%s,%s,%s,%s,%s\n','Id','bb_tl_x','bb_tl_y','bb_lr_x','bb_lr_y');
testImages = 127;

for i=1:testImages
    sol = zeros(12,4);
    img = imread(['test2/image_' num2str(i-1) '.png']);
    try
        fname = [ 'test2/boxes_' num2str(i-1) '.txt'];
        fid = fopen(fname);
        boxes = textscan(fid,'%s');
    catch err
        
    end
    [rows,columns] = size(boxes{1,1});
    
    %for object1
    bbox = step(detector1,img);
    
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        cropimg = single(cropimg);
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features1,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(1,1) = bbox(idx,1);
    sol(1,2) = bbox(idx,2);
    sol(1,3) = bbox(idx,1)+bbox(idx,3);
    sol(1,4) = bbox(idx,2)+bbox(idx,4);
    else
        %In case there is no detection, place a box in the center.
        sol(1,1) = 150;
        sol(1,2) = 200;
        sol(1,3) = 250;
        sol(1,4) = 400;
    end


    %for object2
    bbox = step(detector2,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features2,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(2,1) = bbox(idx,1);
    sol(2,2) = bbox(idx,2);
    sol(2,3) = bbox(idx,1)+bbox(idx,3);
    sol(2,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(2,1) = 150;
        sol(2,2) = 200;
        sol(2,3) = 250;
        sol(2,4) = 400;
    end
    
    bbox = step(detector3,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features3,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(3,1) = bbox(idx,1);
    sol(3,2) = bbox(idx,2);
    sol(3,3) = bbox(idx,1)+bbox(idx,3);
    sol(3,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(3,1) = 150;
        sol(3,2) = 200;
        sol(3,3) = 250;
        sol(3,4) = 400;
    end
    
    bbox = step(detector4,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features4,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(4,1) = bbox(idx,1);
    sol(4,2) = bbox(idx,2);
    sol(4,3) = bbox(idx,1)+bbox(idx,3);
    sol(4,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(4,1) = 150;
        sol(4,2) = 200;
        sol(4,3) = 250;
        sol(4,4) = 400;
    end
    
    bbox = step(detector5,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features5,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(5,1) = bbox(idx,1);
    sol(5,2) = bbox(idx,2);
    sol(5,3) = bbox(idx,1)+bbox(idx,3);
    sol(5,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(5,1) = 150;
        sol(5,2) = 200;
        sol(5,3) = 250;
        sol(5,4) = 400;
    end
    
    bbox = step(detector6,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features6,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(6,1) = bbox(idx,1);
    sol(6,2) = bbox(idx,2);
    sol(6,3) = bbox(idx,1)+bbox(idx,3);
    sol(6,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(6,1) = 150;
        sol(6,2) = 200;
        sol(6,3) = 250;
        sol(6,4) = 400;
    end
    
    bbox = step(detector7,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = (rgb2gray(imcrop(img,bbox(j,:))));
        cropimg = single(cropimg);
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features7,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(7,1) = bbox(idx,1);
    sol(7,2) = bbox(idx,2);
    sol(7,3) = bbox(idx,1)+bbox(idx,3);
    sol(7,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(7,1) = 150;
        sol(7,2) = 200;
        sol(7,3) = 250;
        sol(7,4) = 400;
    end
    
    bbox = step(detector8,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        cropimg = single(cropimg);
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features8,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(8,1) = bbox(idx,1);
    sol(8,2) = bbox(idx,2);
    sol(8,3) = bbox(idx,1)+bbox(idx,3);
    sol(8,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(8,1) = 150;
        sol(8,2) = 200;
        sol(8,3) = 250;
        sol(8,4) = 400;
    end
    
    bbox = step(detector9,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features9,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(9,1) = bbox(idx,1);
    sol(9,2) = bbox(idx,2);
    sol(9,3) = bbox(idx,1)+bbox(idx,3);
    sol(9,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(9,1) = 150;
        sol(9,2) = 200;
        sol(9,3) = 250;
        sol(9,4) = 400;
    end
    
    bbox = step(detector10,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features10,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(10,1) = bbox(idx,1);
    sol(10,2) = bbox(idx,2);
    sol(10,3) = bbox(idx,1)+bbox(idx,3);
    sol(10,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(10,1) = 150;
        sol(10,2) = 200;
        sol(10,3) = 250;
        sol(10,4) = 400;
    end
    
    bbox = step(detector11,img);
    maxmatch = 0;
    idx=1;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features11,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx = j;
        end
    end
    if(size(bbox,1) > 0)
    sol(11,1) = bbox(idx,1);
    sol(11,2) = bbox(idx,2);
    sol(11,3) = bbox(idx,1)+bbox(idx,3);
    sol(11,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(11,1) = 150;
        sol(11,2) = 200;
        sol(11,3) = 250;
        sol(11,4) = 400;
    end
    
    bbox = step(detector12,img);
    maxmatch = 0;
    idx = 1;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features12,d,15);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
            idx=j;
        end
    end
    if(size(bbox,1) > 0)
    sol(12,1) = bbox(idx,1);
    sol(12,2) = bbox(idx,2);
    sol(12,3) = bbox(idx,1)+bbox(idx,3);
    sol(12,4) = bbox(idx,2)+bbox(idx,4);
    else
        sol(12,1) = 150;
        sol(12,2) = 200;
        sol(12,3) = 250;
        sol(12,4) = 400;
    end

    for m=1:rows
        str = textscan(boxes{1,1}{m,1},'%s %d','Delimiter',',');
        id = str{1,2};
        fprintf(out,'%s,%d,%d,%d,%d\n',str{1,1}{1,1},sol(id,1),sol(id,2),sol(id,3),sol(id,4));
%         To View Objects
%         a = [sol(id,1),sol(id,2),sol(id,3)-sol(id,1),sol(id,4)-sol(id,2)];
%         detImg = insertObjectAnnotation(img,'rectangle',a,'objects');
%         figure;
%         imshow(detImg);
    end

end