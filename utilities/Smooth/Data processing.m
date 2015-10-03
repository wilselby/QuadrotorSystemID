%+++ Data processing have five steps,including calculating mean spectra,spectral preprocessing
%    sample division, PLS model building and variable seelction by CARS-PLS model building
%+++ Qianyi Luo, November 17,2014.
%+++ Tutor: Hongmei Lu, Hongmeilu@csu.edu.cn.
%+++ Contact: qianyiluo23@163.com, yunyonghuan@foxmail.com.



%%% Step one - calculating mean spectra 
[m,n]=size(X_ALL); %X_ALL is the data matrix. m is the number of samples, while n is the number of variables
% every sample was detected three times.
X=zeros(m/3,n); 
j=1;
for i=1:3:m
X(j,:)=mean(X_ALL(i:i+2,:));
j=j+1;
end
		
 
%%% Step two - spectral preprocessing, including smooth, SNV, MSC, S/G 1st der
% Moving window smoothing 
[xm]=smooth(X,5);% 5: Spectral window size; X is the data matrix of mean spectra
% SNV(Standard normal transformation) 
[Xsnv]=snv(X);
% MSC(Multiplicative scattering correction) 
[xmsc]=msc(X,1,size(X,2));% 1: first variable used for correction, size(X,2): last variable used for correction
% S/G 1st der(Savitzky-Golay first-derivative) 
[Xde]=deriv(X,1,5,2);% 1:degree of the derivative; 5:Spectral window size; 2:the order of the polynomial

 
%%% Step three - Kennard and Stone algorithm (K-S): samples spilt into calibration and prediction set
ratio=0.8; % Eighty percent of the samples were selected as calibration set and twenty percent as prediction set  
[mx,nx]=size(X);% X: data matrix after data preprocessing. mx is the number of samples, while nx is the number of variables.
mtrain=ceil(mx*ratio);
mtest=mx-mtrain;
[Xtrain,Xtest,Ytrain,Ytest]=ks(X,Y,ceil(mx*ratio));% Y:the target chemical properties matrix


%%% Step four - Partial least squares (PLS) model building
A_max=18; % A_max: the maximal principle component to extract.
fold=10;% fold: the group number for cross validation.
method='center';% method: data pretreatment method, contains: autoscaling, pareto,minmax,center or none.
CV=plscvfold(Xtrain,Ytrain,A_max,fold,method);% cross validation of PLS to select the best PLS component
A=CV.optPC; % The best PLS component 
PLS=pls(Xtrain,Ytrain,A,method);
Xtest_expand=[Xtest ones(size(Xtest,1),1)];
coef=PLS.coef_origin;
ypred=Xtest_expand*coef(:,end);

% Model assessment
SST=sum((Ytest-mean(Ytest)).^2); 
SSE=sum((Ytest-ypred).^2); 
R2_C=PLS.R2; 
R2_P=1-SSE/SST;
RMSEC=sqrt(PLS.SSE/size(Xtrain,1));
RMSEP=sqrt(SSE/size(Xtest,1));
% plot the correlation diagrams between the predicted values and the reference values 
ypred_test = [ypred,Ytest];
plot(Ytrain,PLS.y_est,'*r')
hold on
plot(Ytest,ypred,'ob') % plot the figure

%%% Step five - CARS(competitive adaptive reweighted sampling)-PLS model building
A_max=18; % A_max: the maximal principle component to extract.
fold=10;% fold: the group number for cross validation.
method='center';% method: data pretreatment method, contains: autoscaling, pareto,minmax,center or none.
CV=plscvfold(Xtrain,Ytrain,A_max,fold,method);% cross validation of PLS to select the best PLS component
F=carspls(Xtrain,Ytrain,CV.optPC,fold,method,100);
variables_cars =F.vsel;
XXtrain=Xtrain(:,F.vsel);
XXtest=Xtest(:,F.vsel);
CV=plscvfold(XXtrain,Ytrain,A_max,fold,method,0);
A_cars =CV.optPC; 
PLS=pls(XXtrain,Ytrain,A_cars ,method);
Xtest_expand=[XXtest ones(size(Xtest,1),1)];
coef=PLS.coef_origin;
ypred=Xtest_expand*coef(:,end);

% Model assessment
SST=sum((Ytest-mean(Ytest)).^2);   
SSE=sum((Ytest-ypred).^2);  
R2_C_cars =PLS.R2; 
R2_P_cars =1-SSE/SST;
RMSEC_cars =sqrt(PLS.SSE/size(Xtrain,1));
RMSEP_cars =sqrt(SSE/size(Xtest,1));

% plot the correlation diagrams between the predicted values and the reference values 
ypred_test = [ypred,Ytest];
plot(Ytrain,PLS.y_est,'*r')
hold on
plot(Ytest,ypred,'ob') % plot the figure


