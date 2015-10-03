%#									
%#  function [y] = grampoly(i,m,k,s)	 					
%#									
%#  AIM: 	Derivative computation by using the Savitsky-Golay		 		
%#		algorithm: Weight computation.  			
%#									
%#  PRINCIPLE:  Calculates the Gram Polynomial			 	
%# 									
%#  INPUT:	i	- index of the data point			
%#		m	- index of the filter length		 	
%#		k	- order of the polinomial  			
%#		s	- order of the derivative			
%#									
%#  OUTPUT:	y	- Gram Polynomial vector			
%#									
%#  AUTHOR: 	Luisa Pasti	 				 	
%#	    	Copyright(c) 1997 for ChemoAc				
%#          	FABI, Vrije Universiteit Brussel            		
%#          	Laarbeeklaan 103 1090 Jette				
%#		Modified program of					
%#		Sijmen de Jong						
%#		Unilever Research Laboratorium Vlaardingen		
%#    	    								
%# VERSION: 1.1 (28/02/1998)							 
%#									
%#  TEST:   	Kris De Braekeleer					
%#									

function y=grampoly(i,m,k,s)
if k>0
  r1=grampoly(i,m,k-1,s);
  r2=grampoly(i,m,k-1,s-1);
  r3=grampoly(i,m,k-2,s);
y=((4*k-2)/(k*(2*m-k+1)))*(i*r1+s*r2)-(((k-1)*(2*m+k))/(k*(2*m-k+1)))*r3;
else
  if ((k==0)&(s==0)),y=1;else y=0;end
end
