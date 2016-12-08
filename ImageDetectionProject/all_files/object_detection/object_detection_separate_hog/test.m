out = fopen('out.csv','w');
numImages = 114;
thres = intmin;
cellSize = [8 8];
maxsize = 27000;

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

    windowWidth = 150;
    windowHeight = 150;
    
    j=1;
    i=1;
    while j < imageHeight - windowHeight + 1
        i=1;
        while i < imageWidth - windowWidth + 1
            window = img(j:j + windowHeight - 1, i:i + windowWidth - 1, :);
            %imshow(window);
            testFeatures = extractHOGFeatures(window, 'CellSize', cellSize);
            pad = maxsize - size(testFeatures,2);
            testFeatures = [testFeatures zeros(1,pad)];
            
            label1 = predict(classifier1, testFeatures);
            res = strcmp('1',num2str(label1));
            if res==1
                sol(1)=1;
            end
            
            label2 = predict(classifier2, testFeatures);
            res = strcmp('2',num2str(label2));
            if res==1
                sol(2)=1;
            end
            
            label3 = predict(classifier3, testFeatures);
            res = strcmp('3',num2str(label3));
            if res==1
                sol(3)=1;
            end
            
            label4 = predict(classifier4, testFeatures);
            res = strcmp('4',num2str(label4));
            if res==1
                sol(4)=1;
            end
            
            label5 = predict(classifier5, testFeatures);
            res = strcmp('5',num2str(label5));
            if res==1
                sol(5)=1;
            end
            
            label6 = predict(classifier6, testFeatures);
            res = strcmp('6',num2str(label6));
            if res==1
                sol(6)=1;
            end
            
            label7 = predict(classifier7, testFeatures);
            res = strcmp('7',num2str(label7));
            if res==1
                sol(7)=1;
            end
            
            label8 = predict(classifier8, testFeatures);
            res = strcmp('8',num2str(label8));
            if res==1
                sol(8)=1;
            end
            
            label9 = predict(classifier9, testFeatures);
            res = strcmp('9',num2str(label9));
            if res==1
                sol(9)=1;
            end
            
            label10 = predict(classifier10, testFeatures);
            res = strcmp('10',num2str(label10));
            if res==1
                sol(10)=1;
            end
            
            label11 = predict(classifier11, testFeatures);
            res = strcmp('11',num2str(label11));
            if res==1
                sol(11)=1;
            end
            
            label12 = predict(classifier12, testFeatures);
            res = strcmp('12',num2str(label12));
            if res==1
                sol(12)=1;
            end
            
            i=i+50;
        end
        j=j+50;
    end
    
    for m=1:rows
        str = textscan(boxes{1,1}{m,1},'%s %d','Delimiter',',');
        fprintf(out,'%s,%d\n',str{1,1}{1,1},sol(m));
    end
end