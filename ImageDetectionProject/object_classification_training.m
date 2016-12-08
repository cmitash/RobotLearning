clc;
clear ;
close all;

k = [];
cellSize = [10 10];

numImages = 1058;
%To calculate the MAX size of HOG features
for i = 1:numImages
   img = rgb2gray(imread(['train/image_' num2str(i) '.png']));
    try
        fname = [ 'train/boxes_' num2str(i) '.txt'];
        boxes = csvread(fname);
    catch err
        
    end
    [rows,columns] = size(boxes);
        
    for j=1:rows
        cropimg = imcrop(img,[boxes(j,2) boxes(j,3) boxes(j,4)-boxes(j,2) boxes(j,5)-boxes(j,3)]);
        hogf = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        k = [k;size(hogf,2)];
    end
end

maxsize = max(k);
trainingFeatures = zeros(length(k),maxsize);
trainingLabels = zeros(length(k),1);
count = 1;
%Creating an array of features and lalbels
for i = 1:numImages
   img = rgb2gray(imread(['train/image_' num2str(i) '.png']));
    
    try
     fname = [ 'train/boxes_' num2str(i) '.txt'];
     boxes = csvread(fname);
    catch err
        
    end
    
    [rows,columns] = size(boxes);
        
    for j=1:rows
        cropimg = imcrop(img,[boxes(j,2) boxes(j,3) boxes(j,4)-boxes(j,2) boxes(j,5)-boxes(j,3)]);
        hogf = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        
        pad = maxsize - size(hogf,2);
        hogf = [hogf zeros(1,pad)];
        
        trainingFeatures(count,:) = hogf;
        trainingLabels(count,1) = boxes(j,1);
        count = count+1;
    end
end
%Training using SVM, KNN
classifier1 = fitcknn(trainingFeatures, trainingLabels,'NumNeighbors',4);
classifier2 = fitcecoc(trainingFeatures, trainingLabels);
save('classifierknn1','classifier1','cellSize','maxsize','-v7.3');
save('classifierecoc1','classifier2','cellSize','maxsize','-v7.3');