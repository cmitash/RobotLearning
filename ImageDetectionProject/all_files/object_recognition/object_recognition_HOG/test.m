clc;
clear ;
close all;

out = fopen('out.csv','w');
load('classifier.mat');

maxscore = zeros(12);
sol = zeros(12,4);

sizex = 140;
sizey = 137;

numImages = 127;
fprintf(out,'%s,%s\n','Id','Category');
for i = 1:1
   img = rgb2gray(imread(['test1/image_' num2str(i-1) '.png']));
    try
        fname = [ 'test1/boxes_' num2str(i-1) '.txt'];
        fid = fopen(fname);
        boxes = textscan(fid,'%s');
    catch err
        
    end
    [rows,columns] = size(boxes{1,1});

    for y=1:size(img,2)-sizey
        for x= 1:size(img,1)-sizex
            cropimg = imcrop(img,[x y sizex sizey]);
                
            testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
            pad = maxsize - size(testFeatures,2);
                
            testFeatures = [testFeatures zeros(1,pad)];
            [predictedLabels, score] = predict(classifier, testFeatures);
                
            if score>maxscore(predictedLabels)
                sol(predictedLabels) = [x y x+sizex y+sizey];
                maxscore(predictedLabels) = score;
            end
        end
    end
    
    for j=1:rows
        str = textscan(boxes{1,1}{j,1},'%s %d','Delimiter',',');
        fprintf(out,'%s,%d,%d,%d,%d\n',str{1,1}{1,1},sol(str{1,2},0),sol(str{1,2},1),sol(str{1,2},2),sol(str{1,2},3));
    end
end