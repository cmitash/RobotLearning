out = fopen('out.csv','w');
numImages = 114;
load('classifier8.mat');

fprintf(out,'%s,%s\n','Id','Category');
for k = 1:numImages
   img = rgb2gray(imread(['test1/image_' num2str(k-1) '.png']));
    try
        fname = [ 'test1/boxes_' num2str(k-1) '.txt'];
        fid = fopen(fname);
        boxes = textscan(fid,'%s');
    catch err
        
    end
    [rows,columns] = size(boxes{1,1});
    
    sol = zeros(12);
    imageWidth = size(img, 2);
    imageHeight = size(img, 1);

    windowWidth = 200;
    windowHeight = 200;
    
    j=1;
    i=1;
    while j < imageHeight - windowHeight + 1
        i=1;
        while i < imageWidth - windowWidth + 1
            window = img(j:j + windowHeight - 1, i:i + windowWidth - 1, :);
            imshow(window);
            testFeatures = extractHOGFeatures(window, 'CellSize', cellSize);
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
            [predictedLabels, score] = predict(classifier, testFeatures);
            sol(predictedLabels) = 1;
            predictedLabels
            i=i+50;
        end
        j=j+50;
    end

    for m=1:rows
        str = textscan(boxes{1,1}{m,1},'%s %d','Delimiter',',');
        fprintf(out,'%s,%d\n',str{1,1}{1,1},sol(m));
    end
end