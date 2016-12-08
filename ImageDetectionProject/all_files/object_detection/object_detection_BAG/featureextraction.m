clc;
clear ;
close all;

% for i=1:12
%     mkdir('trainset/',num2str(i));
% end
% 
% numImages = 1056;
% count = 0;
% for i = 1:numImages
%    img = imread(['train/image_' num2str(i-1) '.png']);
%    
%     try
%         fname = [ 'train/boxes_' num2str(i-1) '.txt'];
%         boxes = csvread(fname);
%     catch err
%         
%     end
%     [rows,columns] = size(boxes);
%         
%     for j=1:rows
%         cropimg = imcrop(img,[boxes(j,2) boxes(j,3) boxes(j,4)-boxes(j,2) boxes(j,5)-boxes(j,3)]);
%         baseFileName = sprintf('%d.png', count);
%         count = count+1;
%         fullFileName = fullfile('trainset',num2str(boxes(j,1)), baseFileName);
%         imwrite(cropimg, fullFileName);
%     end
% end

%Collect features from training set
count = 1;
for k=1:1
    rootFolder = fullfile('trainset');
    imgSet = imageSet(fullfile(rootFolder, num2str(k)));
    [trainingSet, validationSet] = partition(imgSet, 0.8, 'randomize');
    for i=1:trainingSet.Count
        im = single(rgb2gray(read(trainingSet,i)));
        [f,d] = vl_sift(im,'PeakThresh', 20) ;
%         spoints = detectSURFFeatures(im);
%         spoints = selectStrongest(spoints, 100);
%         sfeatures = extractFeatures(im, spoints);
         for j =1:size(d,2)
             features1(:,count) = d(:,j);
             count = count+1;
         end
%         for j =1:size(sfeatures,1)
%             features1(count,:) = sfeatures(j,:);
%             count = count+1;
%         end
    end
end

%Validate feature matching
 k=[];
% for i=1:validationSet.Count
%     im = rgb2gray(read(validationSet,i));
%     vpoints = detectSURFFeatures(im);
%     vfeatures = extractFeatures(im, vpoints);
%     boxPairs = matchFeatures(vfeatures, features1);
%     k=[k;size(boxPairs,1)];
% end

for i=1:validationSet.Count
    im = single(rgb2gray(read(validationSet,i)));
    [vf,vd] = vl_sift(im);
    [matches, scores] = vl_ubcmatch(features1,vd) ;
    k=[k;size(matches,2)];
end
save('features1','features1');