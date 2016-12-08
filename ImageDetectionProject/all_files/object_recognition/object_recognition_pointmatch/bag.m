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
        
out = fopen('out.csv','w');
numTestImages = 127;
fprintf(out,'%s,%s\n','Id','Category');
for i = 1:1
   sceneImage = rgb2gray(imread(['test1/image_' num2str(i-1) '.png']));
    try
        fname = [ 'test1/boxes_' num2str(i-1) '.txt'];
        fid = fopen(fname);
        boxes = textscan(fid,'%s');
    catch err
        
    end
    [rows,columns] = size(boxes{1,1});

    for j=1:rows
        str = textscan(boxes{1,1}{j,1},'%s %d','Delimiter',',');
        
        scenePoints = detectSURFFeatures(sceneImage);
        [sceneFeatures, scenePoints] = extractFeatures(sceneImage, scenePoints);
        
        str{1,2}
        imgSets(str{1,2}).Count
        
        maxmatch = 0;
        idx=0;
        %for k=1:imgSets(str{1,2}).Count
        for k=1:10
            boxImage = rgb2gray(read(imgSets(str{1,2}), k));
            boxPoints = detectSURFFeatures(boxImage);
            boxFeatures = extractFeatures(boxImage, boxPoints);
        
            boxPairs = matchFeatures(boxFeatures, sceneFeatures);
            
            try
                figure;
                imshow(boxImage);
                title('100 Strongest Feature Points from Box Image');
                hold on;
                plot(selectStrongest(boxPoints, 100));
            catch
            end
            if size(boxPairs,1)>maxmatch
                maxmatch=size(boxPairs,1);
                idx=k;
            end
        end
        
        boxImage = rgb2gray(read(imgSets(str{1,2}), idx));
        boxPoints = detectSURFFeatures(boxImage);
        [boxFeatures, boxPoints] = extractFeatures(boxImage, boxPoints);
        
        boxPairs = matchFeatures(boxFeatures, sceneFeatures);
        
        matchedBoxPoints = boxPoints(boxPairs(:, 1), :);
        matchedScenePoints = scenePoints(boxPairs(:, 2), :);
        
        [tform, inlierBoxPoints, inlierScenePoints] = ...
            estimateGeometricTransform(matchedBoxPoints, matchedScenePoints, 'affine');
        
        boxPolygon = [1, 1;...                           % top-left
        size(boxImage, 2), 1;...                 % top-right
        size(boxImage, 2), size(boxImage, 1);... % bottom-right
        1, size(boxImage, 1);...                 % bottom-left
        1, 1];                   % top-left again to close the polygon
        
        newBoxPolygon = transformPointsForward(tform, boxPolygon);
    end
end

            