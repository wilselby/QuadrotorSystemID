%#									    
%#  function [sum] = weight(i,t,m,n,s)					
%#									
%#  AIM: 	Derivative computation by using the Savitsky-Golay			 		
%#		algorithm: Weight computation. 		 		
%#									
%#  PRINCIPLE:  Computation of the weight.  				
%# 									
%#  INPUT:	i	- index of the ith data point			
%#		t	- index of the tth Leat Square point of the 	
%#			  s derivative					
%#		m	- the number of points in filter 		
%#		n	- order of the polynomial 		
%#		s	- derivative order				
%#									
%#  OUTPUT:	sum	- Matrix of weight				
%#									
%#  SUBROUTINE:								
%#		genfact							
%#		grampoly						
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

function sum=weight(i,t,m,n,s)

sum=0;
for k=0:n;
sum=sum+(2*k+1)*(genfact(2*m,k)/genfact(2*m+k+1,k+1))*...
grampoly(i,m,k,0)*grampoly(t,m,k,s);
end
