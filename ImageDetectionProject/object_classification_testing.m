clc;
clear ;
close all;

out = fopen('out.csv','w');
load('classifierknn.mat');
load('classifierecoc.mat');

numImages = 112;
fprintf(out,'%s,%s\n','Id','Category');
for i = 1:numImages
   img = rgb2gray(imread(['test2/image_' num2str(i-1) '.png']));
    try
        fname = [ 'test2/boxes_' num2str(i-1) '.txt'];
        fid = fopen(fname);
        boxes = textscan(fid,'%s');
    catch err
        
    end
    [rows,columns] = size(boxes{1,1});
    
    for j=1:rows
        str = textscan(boxes{1,1}{j,1},'%s %d %d %d %d','Delimiter',',');            
        cropimg = imcrop(img,[str{1,2} str{1,3} str{1,4}-str{1,2} str{1,5}-str{1,3}]);
        
        %HOG feature extraction
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        pad = maxsize - size(testFeatures,2);
        testFeatures = [testFeatures zeros(1,pad)];
        
        %Classify based on SVM, KNN
        [predictedLabels1, score1] = predict(classifier1, testFeatures);
        [predictedLabels2, score2] = predict(classifier2, testFeatures);
        
        %Use the better result
        if score1>score2
            predictedLabels = predictedLabels1;
        else
            predictedLabels = predictedLabels2;
        end
        fprintf(out,'%s,%d\n',str{1,1}{1,1},predictedLabels);
    end
end