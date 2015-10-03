%#										
%#  function [xmsc,me,xtmsc]=msc(x,first,last,xt)			
%#									
%#  AIM: 	Multiple Scatter Correction:				
%#		To remove the effect of physical light scatter 		
%#		from the spectrum. (Compensation for particle size	
%#		effects.)						
%#								
%#  PRINCIPLE:  Each spectrum is shifted and rotated so that it fits 	
%#              as closely as possible to the mean spectrum of the data.
%#		The fit is achieved by LS (first-degree polynomial).	
%#		The correction depends on the mean spectrum of the 	
%#		training set.						
%# 									
%#  INPUT:	x: (m x n) matrix with m spectra and n variables	
%#		first: first variable used for correction		
%#		last: last variable used for correction			
%#		 (A segment is selected which is representative for the	
%#		 baseline of the spectra.)				
%#		xt: (mt x nt) matrix for new data (optional)		
%#			 						
%#  OUTPUT:	xmsc: (m x n) matrix containing the spectra after	
%#		    	  correction with msc				
%#		me: mean spectrum (1 x n) of x				
%#		xtmsc: (mt x nt) matrix containing the new spectra after
%#			  correction with msc							
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

function [xmsc,me,xtmsc]=msc(x,first,last,xt);	

if nargin==1;
   first=input('The first variable for the correction: ');
   last=input('The last variables for the correction: ');
end

[m,n]=size(x);
me=mean(x);

for i=1:m,							% for the x data
  p=polyfit(me(first:last),x(i,first:last),1);			% least square fit between mean spectrum and each spectrum (first-degree polynomial)
  xmsc(i,:)=(x(i,:)-p(2)*ones(1,n))./(p(1)*ones(1,n));		% each spectrum is corrected
end

if nargin ==4;							% correction of new data by using the mean spectrum from x.
[mt,nt]=size(xt);			
  for i=1:mt,
    p=polyfit(me(first:last),xt(i,first:last),1);		% least square fit between mean spectrum and each new spectrum (first-degree polynomial)
    xtmsc(i,:)=(xt(i,:)-p(2)*ones(1,n))./(p(1)*ones(1,n));	% each new spectrum is corrected
  end
end

end
