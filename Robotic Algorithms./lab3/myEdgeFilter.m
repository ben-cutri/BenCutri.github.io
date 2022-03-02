function [Im Io Ix Iy] = myEdgeFilter(img,sigma)
G = fspecial('Gaussian',[3 3],sigma);
smimg = myImageFilter(img,G);
%imshow(smimg)
Gx = [-1 0 1;-2 0 2;-1 0 1];
Gy = [1 2 1;0 0 0;-1 -2 -1];
Ix = myImageFilter(smimg,Gx);
Iy = myImageFilter(smimg,Gy);
Im = sqrt(Ix.*Ix + Iy.*Iy);
Io = atan2(Iy,Ix);
for i = 1 : size(Im,1)
    for j = 1 : size(Im,2)
        if Im(i,j) < 50
            Im(i,j) = 0;
        end
    end
end
%Im%=floor(Im);
imshow(Im);


end


