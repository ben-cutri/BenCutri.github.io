function [] = main()
    for x = 1:5
    str = int2str(x);
    file = strcat('img0',str,'.jpg');
    sigma = 5;
    threshold = 10000000;
    img0=imread(file);
    [x,y] = size(img0);
    img1 = zeros(x,y);
    img1 = img0;
    d=ndims(img1);
    if d>2
        img1=rgb2gray(img0);
    end
    [Im Io Ix Iy] = myEdgeFilter(img1,sigma);
    R = myHarrisCorner(Ix,Iy,threshold);
    a = figure;
    %figure;
    %imshow(Im);
    imshow(uint8(img0));
    hold on;
    plot(R(:,2),R(:,1),'o')            
    hold off;
    output = strcat('outputimg0',str,'.jpg');
    saveas(a,output);
end
end
