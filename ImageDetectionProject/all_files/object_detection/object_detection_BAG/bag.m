clc;
clear ;
close all;

for i=1:12
    mkdir('trainset/',num2str(i));
end

numImages = 1056;
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

for k=1:12
    delete('trainset/neg/*');
    for i=1:12
        if i==k
            continue;
        end
        copyfile(['trainset/' num2str(i) '/*'],'trainset/neg/');    
    end

    rootFolder = fullfile('trainset');
    imgSets = [ imageSet(fullfile(rootFolder, num2str(k))), ...
                imageSet(fullfile(rootFolder, 'neg'))];

    minSetCount = min([imgSets.Count]);
    imgSets = partition(imgSets, minSetCount, 'randomize');

    %[trainingSets, validationSets] = partition(imgSets, 0.45, 'randomize');

    bagf = bagOfFeatures(imgSets);
    categoryClassifier = trainImageCategoryClassifier(imgSets, bagf);
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

            