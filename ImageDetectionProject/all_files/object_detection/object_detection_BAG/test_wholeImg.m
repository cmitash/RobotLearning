out = fopen('out.csv','w');
numImages = 114;
thres = -0.4;

fprintf(out,'%s,%s\n','Id','Category');
for k = 1:numImages
   img = imread(['test1/image_' num2str(k-1) '.png']);
    try
        fname = [ 'test1/boxes_' num2str(k-1) '.txt'];
        fid = fopen(fname);
        boxes = textscan(fid,'%s');
    catch err
        
    end
    [rows,columns] = size(boxes{1,1});
    
    sol = zeros(12);
 
    [label1, score]= predict(classifier1, img);
    res = strcmp('1',num2str(label1));
    if res==1 && score(1)>thres
        sol(1)=1;
    end

    [label2, score] = predict(classifier2, img);
    res = strcmp('1',num2str(label2));
    if res==1 && score(1)>thres
        sol(2)=1;
    end

    [label3, score] = predict(classifier3, img);
    res = strcmp('1',num2str(label3));
    if res==1 && score(1)>thres
        sol(3)=1;
    end

    [label4, score] = predict(classifier4, img);
    res = strcmp('1',num2str(label4));
    if res==1 && score(1)>thres
        sol(4)=1;
    end

    [label5, score] = predict(classifier5, img);
    res = strcmp('1',num2str(label5));
    if res==1 && score(1)>thres
        sol(5)=1;
    end

    [label6, score] = predict(classifier6, img);
    res = strcmp('1',num2str(label6));
    if res==1 && score(1)>thres
        sol(6)=1;
    end

    [label7, score] = predict(classifier7, img);
    res = strcmp('1',num2str(label7));
    if res==1 && score(1)>thres
        sol(7)=1;
    end

    [label8, score] = predict(classifier8, img);
    res = strcmp('1',num2str(label8));
    if res==1 && score(1)>thres
        sol(8)=1;
    end

    [label9, score] = predict(classifier9, img);
    res = strcmp('1',num2str(label9));
    if res==1 && score(1)>thres
        sol(9)=1;
    end

    [label10, score] = predict(classifier10, img);
    res = strcmp('1',num2str(label10));
    if res==1 && score(1)>thres
        sol(10)=1;
    end

    [label11, score] = predict(classifier11, img);
    res = strcmp('1',num2str(label11));
    if res==1 && score(1)>thres
        sol(11)=1;
    end

    [label12, score] = predict(classifier12, img);
    res = strcmp('1',num2str(label12));
    if res==1 && score(1)>thres
        sol(12)=1;
    end
            
    for m=1:rows
        str = textscan(boxes{1,1}{m,1},'%s %d','Delimiter',',');
        fprintf(out,'%s,%d\n',str{1,1}{1,1},sol(m));
    end
end