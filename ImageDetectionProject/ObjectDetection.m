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
detector1 = vision.CascadeObjectDetector('obj1.xml','MinSize',[50 50],'MaxSize',[150 150]);
detector2 = vision.CascadeObjectDetector('obj2.xml','MinSize',[50 50],'MaxSize',[130 100]);
detector3 = vision.CascadeObjectDetector('obj3.xml','MinSize',[50 50],'MaxSize',[250 200]);
detector4 = vision.CascadeObjectDetector('obj4.xml','MinSize',[75 75],'MaxSize',[200 200]);
detector5 = vision.CascadeObjectDetector('obj5.xml','MinSize',[50 50],'MaxSize',[150 150]);
detector6 = vision.CascadeObjectDetector('obj6.xml','MinSize',[50 75],'MaxSize',[200 150]);
detector7 = vision.CascadeObjectDetector('obj7.xml','MinSize',[50 50],'MaxSize',[120 150]);
detector8 = vision.CascadeObjectDetector('obj8.xml','MinSize',[50 50],'MaxSize',[200 250]);
detector9 = vision.CascadeObjectDetector('obj9.xml','MinSize',[75 75],'MaxSize',[250 200]);
detector10 = vision.CascadeObjectDetector('obj10.xml','MinSize',[75 75],'MaxSize',[150 200]);
detector11 = vision.CascadeObjectDetector('obj11.xml','MinSize',[75 75],'MaxSize',[150 200]);
detector12 = vision.CascadeObjectDetector('obj12.xml','MinSize',[75 75],'MaxSize',[150 200]);

fprintf(out,'%s,%s\n','Id','Category');
%Set the learned thresholds
thres = [15 10 15 10 15 10 20 20 40 15 15 10];
testImages = 126;
for i=1:testImages
    sol = zeros(1,12);
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
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        cropimg = single(cropimg);
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features1,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(1))
        sol(1) = 1;
    end

    %for object2
    bbox = step(detector2,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features2,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(2))
        sol(2) = 1;
    end

    %for object3
    bbox = step(detector3,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features3,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(3))
        sol(3) = 1;
    end

    %for object4
    bbox = step(detector4,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features4,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(4))
        sol(4) = 1;
    end

    %for object5
    bbox = step(detector5,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features5,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(5))
        sol(5) = 1;
    end

    %for object6
    bbox = step(detector6,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features6,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(6))
        sol(6) = 1;
    end

    %for object7
    bbox = step(detector7,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = (rgb2gray(imcrop(img,bbox(j,:))));
        cropimg = single(cropimg);
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features7,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(7))
        sol(7) = 1;
    end

    %for object8
    bbox = step(detector8,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features8,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(8))
        sol(8) = 1;
    end

    %for object9
    bbox = step(detector9,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features9,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(9))
        sol(9) = 1;
    end

    %for object10
    bbox = step(detector10,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features10,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(10))
        sol(10) = 1;
    end

    %for object11
    bbox = step(detector11,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features11,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(11))
        sol(11) = 1;
    end

    %for object12
    bbox = step(detector12,img);
    maxmatch = 0;
    for j=1:size(bbox,1)
        cropimg = single(rgb2gray(imcrop(img,bbox(j,:))));
        [f,d] = vl_sift(cropimg);
        [matches, scores] = vl_ubcmatch(features12,d,10);
        if(size(matches,2) > maxmatch)
            maxmatch = size(matches,2);
        end
    end
    if(maxmatch > thres(12))
        sol(12) = 1;
    end

    for m=1:rows
        str = textscan(boxes{1,1}{m,1},'%s %d','Delimiter',',');
        fprintf(out,'%s,%d\n',str{1,1}{1,1},sol(m));
    end

end