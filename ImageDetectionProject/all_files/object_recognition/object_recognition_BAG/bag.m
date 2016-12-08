clc;
clear ;
close all;

for i=1:12
    mkdir('trainset/',num2str(i));
end

numImages = 1023;
count = 0;
for i = 1:numImages
   img = imread(['train/image_' num2str(i-1) '.png']);
    try
        fname = [ 'train/boxes_' num2str(i-1) '.txt'];
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

rootFolder = fullfile('trainset');
imgSets = [ imageSet(fullfile(rootFolder, '1')), ...
            imageSet(fullfile(rootFolder, '2')), ...
            imageSet(fullfile(rootFolder, '3')), ...
            imageSet(fullfile(rootFolder, '4')), ...
            imageSet(fullfile(rootFolder, '5')), ...
            imageSet(fullfile(rootFolder, '6')), ...
            imageSet(fullfile(rootFolder, '7')), ...
            imageSet(fullfile(rootFolder, '8')), ...
            imageSet(fullfile(rootFolder, '9')), ...
            imageSet(fullfile(rootFolder, '10')), ...
            imageSet(fullfile(rootFolder, '11')), ...
            imageSet(fullfile(rootFolder, '12'))];
        
minSetCount = min([imgSets.Count]);
imgSets = partition(imgSets, minSetCount, 'randomize');

bagf = bagOfFeatures(imgSets);
categoryClassifier = trainImageCategoryClassifier(imgSets, bagf);




            