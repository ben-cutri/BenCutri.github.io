function [R] = myHarrisCorner(Ix,Iy,threshold)
k = 0.04;
sigma = 5;
Ix2 = Ix .* Ix;
Iy2 = Iy .* Iy;
Ixy = Ix .* Iy;
G = fspecial('Gaussian',[3 3],sigma);
Sx2 = myImageFilter(Ix2,G);
Sy2 = myImageFilter(Iy2,G);
Sxy = myImageFilter(Ixy,G);
for x = 1 : size(Ix2,1)
    for y = 1 : size(Iy2,2)
        M = [Sx2(x,y) Sxy(x,y);Sxy(x,y) Sy2(x,y)];
        r(x,y) = det(M) - k * (trace(M) ^ 2);
    end
end
%max(max(r))
r = r > threshold;
R = zeros(1,2);
for i = 1:size(r,1)
    for j = 1:size(r,2)
        if r(i,j) == 1
            R = [R;[i j]];
        end
    end
end
end

          
