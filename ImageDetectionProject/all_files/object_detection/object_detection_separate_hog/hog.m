clc;
clear ;
close all;
cellSize = [8 8];
maxsize = 27000;

for i=1:12
    mkdir('trainset2/',num2str(i));
end

numImages = 1056;
count = 0;
for i = 1:numImages
   img = rgb2gray(imread(['train/image_' num2str(i-1) '.png']));
   
    try
        fname = [ 'train/boxes_' num2str(i-1) '.txt'];
        boxes = csvread(fname);
    catch err
        
    end
    [rows,columns] = size(boxes);
        
    for j=1:rows
        xdim = boxes(j,4)-boxes(j,2);
        ydim = boxes(j,5)-boxes(j,3);
        
        if(xdim < 150)
            toex = 150-xdim;
            if(boxes(j,2) > toex)
                boxes(j,2) = boxes(j,2)-toex;
            else
                boxes(j,4) = boxes(j,4)+toex;
            end
        else
            tocon = xdim - 150;
            mid = tocon/2;
            rem = tocon - mid;
            boxes(j,2) = boxes(j,2)+mid;
            boxes(j,4) = boxes(j,4)-rem;
        end
        
        if(ydim < 150)
            toex = 150-ydim;
            if(boxes(j,3) > toex)
                boxes(j,3) = boxes(j,3)-toex;
            else
                boxes(j,5) = boxes(j,5)+toex;
            end
        else
            tocon = ydim - 150;
            mid = tocon/2;
            rem = tocon - mid;
            boxes(j,3) = boxes(j,3)+mid;
            boxes(j,5) = boxes(j,5)-rem;
        end
        cropimg = imcrop(img,[boxes(j,2) boxes(j,3)  boxes(j,4)-boxes(j,2) boxes(j,5)-boxes(j,3)]);
        baseFileName = sprintf('%d.png', count);
        count = count+1;
        fullFileName = fullfile('trainset2',num2str(boxes(j,1)), baseFileName);
        imwrite(cropimg, fullFileName);
    end
end

for k=1:12
    delete('trainset2/neg/*');
    for i=1:12
        if i==k
            continue;
        end
        copyfile(['trainset2/' num2str(i) '/*'],'trainset2/neg/');    
    end

    rootFolder = fullfile('trainset2');
    imgSets = [ imageSet(fullfile(rootFolder, num2str(k))), ...
                imageSet(fullfile(rootFolder, 'neg'))];

    minSetCount = min([imgSets.Count]);
    imgSets = partition(imgSets, minSetCount, 'randomize');
    
    count = 1;
    trainingFeatures = zeros(minSetCount,maxsize);
    trainingLabels = zeros(minSetCount,1);
    
    for a=1:minSetCount
        im = read(imgSets(1),a);
        %imshow(im);
        hogf = extractHOGFeatures(im, 'CellSize', cellSize);
        pad = maxsize - size(hogf,2);
        hogf = [hogf zeros(1,pad)];
        
        trainingFeatures(count,:) = hogf;
        trainingLabels(count,1) = k;
        
        count = count+1;
        
        im = read(imgSets(2),a);
        %imshow(im);
        hogf = extractHOGFeatures(im, 'CellSize', cellSize);
        pad = maxsize - size(hogf,2);
        hogf = [hogf zeros(1,pad)];
        
        trainingFeatures(count,:) = hogf;
        trainingLabels(count,1) = 12+k;
        
        count = count+1;
    end
    categoryClassifier = fitcsvm(trainingFeatures, trainingLabels);
    
    save(['class' num2str(k)],'categoryClassifier');
end

load('class1.mat');
classifier1 = categoryClassifier;
clearvars categoryClassifier;

load('class2.mat');
classifier2 = categoryClassifier;
clearvars categoryClassifier;

load('class3.mat');
classifier3 = categoryClassifier;
clearvars categoryClassifier;

load('class4.mat');
classifier4 = categoryClassifier;
clearvars categoryClassifier;

load('class5.mat');
classifier5 = categoryClassifier;
clearvars categoryClassifier;

load('class6.mat');
classifier6 = categoryClassifier;
clearvars categoryClassifier;

load('class7.mat');
classifier7 = categoryClassifier;
clearvars categoryClassifier;

load('class8.mat');
classifier8 = categoryClassifier;
clearvars categoryClassifier;

load('class9.mat');
classifier9 = categoryClassifier;
clearvars categoryClassifier;

load('class10.mat');
classifier10 = categoryClassifier;
clearvars categoryClassifier;

load('class11.mat');
classifier11 = categoryClassifier;
clearvars categoryClassifier;

load('class12.mat');
classifier12 = categoryClassifier;            