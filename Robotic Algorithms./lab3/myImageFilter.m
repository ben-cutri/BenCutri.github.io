function [img1] = myImageFilter(img0,h)
m_img = size(img0,1);
n_img = size(img0,2);
m_h = size(h,1);
n_h = size(h,2);
k = rot90(h, 2);
mid = floor((size(k)+1)/2);
l = mid(2) - 1;
r = n_h - mid(2);
t = mid(1) - 1;
b = m_h - mid(1);
pad = zeros(m_img + t + b, n_img + l + r);
for i = 1 + t : m_img + t
    for j = 1 + l : n_img + l
        pad(i,j) = img0(i - t, j - l);................................................................................
    end
end
img1 = zeros(m_img , n_img);
for x = 1 : m_img
    for y = 1 : n_img
        for i = 1 : m_h
            for j = 1 : n_h
                a = x - 1;
                b = y - 1;
                img1(x, y) = img1(x, y) + (pad(i + a, j + b) * k(i, j));
            end
        end
    end
end
%imshow(img1)            
end