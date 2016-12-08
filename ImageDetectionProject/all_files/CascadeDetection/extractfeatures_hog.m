clc;
clear ;
close all;

out = fopen('out.csv','w');

load('classifierknn.mat');
maxsize = 15876;
cellSize = [10 10];

detector1 = vision.CascadeObjectDetector('obj1.xml');
detector2 = vision.CascadeObjectDetector('obj2.xml');
detector3 = vision.CascadeObjectDetector('obj3.xml');
detector4 = vision.CascadeObjectDetector('obj4.xml');
detector5 = vision.CascadeObjectDetector('obj5.xml');
detector6 = vision.CascadeObjectDetector('obj6.xml');
detector7 = vision.CascadeObjectDetector('obj7.xml');
detector8 = vision.CascadeObjectDetector('obj8.xml');
detector9 = vision.CascadeObjectDetector('obj9.xml');
detector10 = vision.CascadeObjectDetector('obj10.xml');
detector11 = vision.CascadeObjectDetector('obj11.xml');
detector12 = vision.CascadeObjectDetector('obj12.xml');

fprintf(out,'%s,%s\n','Id','Category');
thres = [-0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1];
testImages = 114;
for i=1:testImages
    sol = zeros(1,12);
    img = imread(['test1/image_' num2str(i-1) '.png']);
    try
        fname = [ 'test1/boxes_' num2str(i-1) '.txt'];
        fid = fopen(fname);
        boxes = textscan(fid,'%s');
    catch err
        
    end
    [rows,columns] = size(boxes{1,1});
    i
    %detImg = insertObjectAnnotation(img,'rectangle',bbox,'crayola');
    %figure;
    %imshow(detImg);
    
    %for object1
    bbox = step(detector1,img);
    k=1;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        imshow(cropimg);
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures)
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object2
    bbox = step(detector2,img);
    k=2;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object3
    bbox = step(detector3,img);
    k=3;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object4
    bbox = step(detector4,img);
    k=4;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object5
    bbox = step(detector5,img);
    k=5;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object6
    bbox = step(detector6,img);
    k=6;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object7
    bbox = step(detector7,img);
    k=7;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object8
    bbox = step(detector8,img);
    k=8;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object9
    bbox = step(detector9,img);
    k=9;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object10
    bbox = step(detector10,img);
    k=10;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object11
    bbox = step(detector11,img);
    k=11;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    %for object12
    bbox = step(detector12,img);
    k=12;
    maxmatch = intmin;
    pred = 0;
    for j=1:size(bbox,1)
        cropimg = rgb2gray(imcrop(img,bbox(j,:)));
        testFeatures = extractHOGFeatures(cropimg, 'CellSize', cellSize);
        if(size(testFeatures,2) > maxsize)
            testFeatures = testFeatures(1:maxsize);
        else
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
        end
        
        [predictedLabels, score] = predict(classifier1, testFeatures);
        if(predictedLabels==k)
            pred = 1;
        end
        
        if(score(k) > maxmatch)
            maxmatch = score(k);
        end
    end
    if(maxmatch > thres(k) && pred==1)
        sol(k) = 1;
    end
    
    for m=1:rows
        str = textscan(boxes{1,1}{m,1},'%s %d','Delimiter',',');
        fprintf(out,'%s,%d\n',str{1,1}{1,1},sol(m));
    end

end