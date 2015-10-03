%#									
%#  function [xsnv]=snv(x)						
%#									
%#  AIM: 	Standard Normal Variate Transformation			
%#		Row centering, followed by row scaling.			
%#									
%#  PRINCIPLE:  Removal of the row mean from each row, followed 	
%#              by division of the row by the respective row	 	
%#		standard deviation.		 		 	
%# 									
%#  INPUT:	x: (m x n) matrix with m spectra and n variables	
%#			 						
%#  OUTPUT:	xsnv: (m x n) matrix containing snv transformed spectra	
%#									
%#  AUTHOR: 	Andrea Candolfi				 		
%#	    	Copyright(c) 1997 for ChemoAC				
%#          	FABI, Vrije Universiteit Brussel            		
%#          	Laarbeeklaan 103 1090 Jette				
%#    	    								
%# VERSION: 1.1 (28/02/1998)						
%#									
%#  TEST:   	Roy de Maesschalck					
%#									

function [xsnv]=snv(x);	

[m,n]=size(x);
xsnv=(x-mean(x')'*ones(1,n))./(std(x')'*ones(1,n));
