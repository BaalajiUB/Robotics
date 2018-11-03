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
## @deftypefn {} {@var{retval} =} im_align1 (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: dell <dell@DELL-PC>
## Created: 2018-10-22

function[gx,gy,rx,ry] = im_align1(R,G,B)
   disp("ssd");
   [x y] = size(B);
   gx =500; gy=500;
   %disp('Green');
   min=Inf;
   bdr = 30;
   for a = -15:15
     for b = -15:15
       t = sum(sum((B(bdr:size(B,1)-bdr,bdr:size(B,2)-bdr)-G(bdr+a:size(B,1)-bdr+a,bdr+b:size(B,2)-bdr+b)).^2));
       if t<min
         min=t;
         gx=a;gy=b;
       end
     end
   end  
   %disp([gx gy]);
   if gx==0
     gx=1;
   end
   if gy==0
     gy=1;
   end

   rx =500; ry=500;   
   %disp('Red');
   min=Inf;
   for a = -15:15
     for b = -15:15
       t = sum(sum((B(bdr:size(B,1)-bdr,bdr:size(B,2)-bdr)-R(bdr+a:size(B,1)-bdr+a,bdr+b:size(B,2)-bdr+b)).^2));
       #t = sum(sum((B(85:255,85:300)-R(85+a:255+a,85+b:300+b)).^2));
       if t<min
         min=t;
         rx=a;ry=b;
       end
     end
   end  
   %disp([rx ry]);   
   if rx==0
     rx=1;
   end
   if ry==0
     ry=1;
   end

endfunction
