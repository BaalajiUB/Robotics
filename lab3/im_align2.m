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
## @deftypefn {} {@var{retval} =} im_align2 (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: dell <dell@DELL-PC>
## Created: 2018-10-22

function[gx,gy,rx,ry] = im_align2(R,G,B)
   %https://octave.org/doc/v4.2.1/Using-Packages.html
   %https://www.mathworks.com/help/images/registering-an-image-using-normalized-cross-correlation.html
   %pkg load image
   disp("ncc");
   
   %disp('Green');

  G_sub = G(100:150,100:150);
  c=normxcorr2(G_sub,B);
  [ypeak, xpeak] = find(c==max(c(:)));

  xoffSet = 100-(ypeak-size(G_sub,1));
  yoffSet = 100-(xpeak-size(G_sub,2));
  %disp([xoffSet yoffSet]);

  if yoffSet==0
    yoffSet=1;  
  endif

  if xoffSet==0
    xoffSet=1;  
  endif

  gx=xoffSet;
  gy=yoffSet;

  R_sub = R(100:150,100:150);  
  c=normxcorr2(R_sub,B);
  [ypeak, xpeak] = find(c==max(c(:)));

  xoffSet = 100-(ypeak-size(R_sub,1));
  yoffSet = 100-(xpeak-size(R_sub,2));

  if yoffSet==0
    yoffSet=1;  
  endif

  if xoffSet==0
    xoffSet=1;  
  endif

  rx=xoffSet;
  ry=yoffSet;
endfunction
