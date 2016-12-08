clc;
clear ;
close all;

for i=1:12
    mkdir('trainset/',num2str(i));
end

%Separate images for each Object
numImages = 1056;
count = 0;
for i = 1:numImages
   img = imread(['train1/image_' num2str(i-1) '.png']);
   
    try
        fname = [ 'train1/boxes_' num2str(i-1) '.txt'];
        boxes = csvread(fname);
    catch err
        
    end
    [rows,columns] = size(boxes);
        
    for j=1:rows
        cropimg = imcrop(img,[boxes(j,2) boxes(j,3) boxes(j,4)-boxes(j,2) boxes(j,5)-boxes(j,3)]);
        baseFileName = sprintf('%d.png', count);
        count = count+1;
        fullFileName = fullfile('trainset',num2str(boxes(j,1)), baseFileName);
        imwrite(cropimg, fullFileName);
    end
end

%Collect features from training set
count = 1;
k=1;
rootFolder = fullfile('trainset');
imgSet = imageSet(fullfile(rootFolder, num2str(k)));
[trainingSet, validationSet] = partition(imgSet, 0.8, 'randomize');
for i=1:imgSet.Count
    im = single(rgb2gray(read(imgSet,i)));
    [f,d] = vl_sift(im) ;
    
     for j =1:size(d,2)
         features1(:,count) = d(:,j);
         count = count+1;
     end
end


%Validate feature matching
for i=1:validationSet.Count
    im = rgb2gray(read(validationSet,i));
    vpoints = detectSURFFeatures(im);
    vfeatures = extractFeatures(im, vpoints);
    boxPairs = matchFeatures(vfeatures, features1);
    k=[k;size(boxPairs,1)];
end
p=[];
for i=1:validationSet.Count
    im = single(rgb2gray(read(validationSet,i)));
    [vf,vd] = vl_sift(im);
    [matches, scores] = vl_ubcmatch(features1,vd,10) ;
    p=[p;size(matches,2)];
end

%Tune Threshold
folder = 1;
l=[];
rootFolder = fullfile('trainset');
load('features1.mat');
imgSet2 = imageSet(fullfile(rootFolder, num2str(folder)));
siz = zeros(imgSet2.Count,2);
for i=1:imgSet2.Count
    im = rgb2gray(read(imgSet2,i));
    siz(i,1) = size(im,1);
    siz(i,2) = size(im,2);
    im = single(im);
    [vf,vd] = vl_sift(im);
    [matches, scores] = vl_ubcmatch(features1,vd,10) ;
    l=[l;size(matches,2)];
end
m = l>20;
sum(m)

save('features1','features1');