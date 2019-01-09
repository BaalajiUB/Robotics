## Copyright (C) 2018 dell
## 
## This program is free software: you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful, but
## WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see
## <https://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {} {@var{retval} =} main (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: dell <dell@DELL-PC>
## Created: 2018-10-22


function main ()
  t=cputime;
  for i = 1:6
     disp(i);
     path =  strcat(strcat("image",int2str(i)),'.jpg'); 
     image = imread(path);
     [x y] = size(image);
     img = zeros(floor(x/3),y,3); %initializing 3 planes as matrices
     j=1;
     B = image(1 + floor(x/3)*(j-1):floor(x/3)*(j),:);
     j=2;
     G = image(1 + floor(x/3)*(j-1):floor(x/3)*(j),:);
     j=3;
     R = image(1 + floor(x/3)*(j-1):floor(x/3)*(j),:);
     img(:,:,1) = R;
     img(:,:,2) = G;
     img(:,:,3) = B;
     %the 3rd argument of img mentions plane number 
     %-- , for horizontal concatenation.-- ; for vertical concatenation
     img = uint8(img); %to convert it to color image
     %figure;
     %imshow(img); %to display the image
     imwrite(img,strcat(strcat("image",int2str(i)),'-color.jpg')); %to save the image
     
     [gx,gy,rx,ry] = im_align1(R,G,B);
     disp([gx gy rx ry]);
     
     [x y] = size(B);
     B_sub = zeros(x,y,3);
     B_sub(:,:,3) = B;
     
     if gx<0 && gy<0
       G_sub = G(1:(x-abs(gx)),1:(y-abs(gy)));
       B_sub(abs(gx)+1:x,abs(gy)+1:y,2) = G_sub;
     elseif gx<0 && gy>=0
       G_sub = G(1:(x-abs(gx)),gy:y);
       B_sub(abs(gx)+1:x,1:y-gy+1,2) = G_sub;
     elseif gx>=0 && gy<0
       G_sub = G(gx:x,1:(y-abs(gy)));
       B_sub(1:x-gx+1,abs(gy)+1:y,2) = G_sub;
     elseif gx>=0 && gy>=0
       G_sub = G(gx:x,gy+1:y);
       B_sub(1:x-gx+1,1:y-gy,2) = G_sub;       
     endif

     if rx<0 && ry<0
       R_sub = R(1:(x-abs(rx)),1:(y-abs(ry)));
       B_sub(abs(rx)+1:x,abs(ry)+1:y,1) = R_sub;
     elseif rx<0 && ry>=0
       R_sub = R(1:(x-abs(rx)),ry:y);
       B_sub(abs(rx)+1:x,1:y-ry+1,1) = R_sub;
     elseif rx>=0 && ry<0
       R_sub = R(rx:x,1:(y-abs(ry)));
       B_sub(1:x-rx+1,abs(ry)+1:y,1) = R_sub;
     elseif rx>=0 && ry>=0
       R_sub = R(rx:x,ry+1:y);
       B_sub(1:x-rx+1,1:y-ry,1) = R_sub;       
     endif
     
     B_sub = uint8(B_sub);
     %figure;
     %imshow(B_sub);
     imwrite(B_sub,strcat(strcat("image",int2str(i)),'-ssd.jpg')); %to save the image

     [gx,gy,rx,ry] = im_align2(R,G,B);
     disp([gx gy rx ry]);
     
     [x y] = size(B);
     B_sub = zeros(x,y,3);
     B_sub(:,:,3) = B;
     
     if gx<0 && gy<0
       G_sub = G(1:(x-abs(gx)),1:(y-abs(gy)));
       B_sub(abs(gx)+1:x,abs(gy)+1:y,2) = G_sub;
     elseif gx<0 && gy>=0
       G_sub = G(1:(x-abs(gx)),gy:y);
       B_sub(abs(gx)+1:x,1:y-gy+1,2) = G_sub;
     elseif gx>=0 && gy<0
       G_sub = G(gx:x,1:(y-abs(gy)));
       B_sub(1:x-gx+1,abs(gy)+1:y,2) = G_sub;
     elseif gx>=0 && gy>=0
       G_sub = G(gx:x,gy+1:y);
       B_sub(1:x-gx+1,1:y-gy,2) = G_sub;       
     endif

     if rx<0 && ry<0
       R_sub = R(1:(x-abs(rx)),1:(y-abs(ry)));
       B_sub(abs(rx)+1:x,abs(ry)+1:y,1) = R_sub;
     elseif rx<0 && ry>=0
       R_sub = R(1:(x-abs(rx)),ry:y);
       B_sub(abs(rx)+1:x,1:y-ry+1,1) = R_sub;
     elseif rx>=0 && ry<0
       R_sub = R(rx:x,1:(y-abs(ry)));
       B_sub(1:x-rx+1,abs(ry)+1:y,1) = R_sub;
     elseif rx>=0 && ry>=0
       R_sub = R(rx:x,ry+1:y);
       B_sub(1:x-rx+1,1:y-ry,1) = R_sub;       
     endif
     
     B_sub = uint8(B_sub);
     %figure;
     %imshow(B_sub);
     imwrite(B_sub,strcat(strcat("image",int2str(i)),'-ncc.jpg')); %to save the image
     
  endfor
  disp(cputime-t);
endfunction

clc
clear all
close all

main()
