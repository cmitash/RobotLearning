clc;
clear ;
close all;

%Generate positive and negative images for training Cascade Detection
for a=1:12;
numImages = 1056;
count = 0;
namecount=0;
for i = 1:numImages
   imname = ['train1/image_' num2str(i-1) '.png'];
   img = imread(imname);
   
    try
        fname = [ 'train1/boxes_' num2str(i-1) '.txt'];
        boxes = csvread(fname);
    catch err
        
    end
    [rows,columns] = size(boxes);
    flag=0;
    
    for j=1:rows
        if(boxes(j,1) == a)
            count = count+1;
            if boxes(j,2) == 0
                boxes(j,2)=1;
            end
            
            if boxes(j,3) == 0
                boxes(j,3)=1;
            end
            
            bbox = [boxes(j,2) boxes(j,3) boxes(j,4)-boxes(j,2) boxes(j,5)-boxes(j,3)];
            data(count).imageFilename = imname;
            data(count).objectBoundingBoxes = bbox;
            
            flag=1;
            break;  
        end
    end
    
    if flag==0
        baseFileName = sprintf('%d.png', namecount);
        namecount = namecount+1;
        fullFileName = fullfile(['trainset/' num2str(a)], baseFileName);
        imwrite(img, fullFileName);
    end
end

save(['data' num2str(a) '.mat'],'data');
end

%Train 12 Cascade Detectors
for b=1:12;
 load(['data' num2str(b) '.mat']);
 negfolder = fullfile('trainset', num2str(b));
 trainCascadeObjectDetector(['obj' num2str(b) '.xml'],data,negfolder,'NumCascadeStages',5);
end