% This file reads the data and splits it into training,
% validation and test datasets.
tic;
filename = 'movements_day1-3.dat';
[Movements,delimiterOut] = importdata(filename);
% Subtract arithmetic mean for each sensor. We only care about how it varies:

Movements(:,1:40) = Movements(:,1:40) - repmat(mean(Movements(:,1:40)),[447 1]); 

% Find maximum absolute value:
imax = max(max(Movements),abs(min(Movements)));

% Divide by imax, values should now be between -1,1
for i = 1: 40
    Movements(:,i) = Movements(:,i)./imax(i);
end

% Generate target vectors for all inputs 2 -> [0,1,0,0,0,0,0,0]
[m,n] = size(Movements)   ; 
target_class=zeros(m,8);
for x=1:8
indices = find(x==Movements(:,41));
target_class(indices,x)=1;
end

% Randomly order the data
randomorder = randperm(m,m);
Movements = Movements(randomorder(),:);
target_class = target_class(randomorder(),:);

% Create "train" and "traintargets" datasets
% Training updates the weights of the network and thus improves the network
train1 = Movements(1:2:end,:);
traintargetsTrans = target_class(1:2:end,:);

numOfSamples = sum(traintargetsTrans);

% Create "valid" and "validtargets" datasets
% Validation checks how well the network is performing and when to stop
valid = Movements(2:4:end,:);
validtargets = target_class(2 :4:end,:);

% Create "test" and "testtargets" datasets
% Test data is used to evaluate how good the completely trained network is.
test = Movements(4:4:end,:);
testtargets = target_class(4:4:end,:);

net = feedforwardnet(32,'traingd') ; 

% net.adaptFcn = 'adaptwb';
net.divideFcn = 'divideind'; %Set the divide function to dividerand (divide training data randomly).

net.performFcn = 'mse';

train1 = train1(:,1:40) ;
traintargetsTrans = traintargetsTrans ;

valid = valid(:,1:40) ;
validtargetsTrans = validtargets;

test = test(:,1:40);
testtargetsTrans = testtargets;

totalInput = [ train1', valid', test']';
totalOutput =[traintargetsTrans', validtargetsTrans', testtargetsTrans']';

trainInd = 1:224;
valInd = 225:336;
testInd = 337:447;
net.divideFcn = 'divideind';
net.divideParam.trainInd = trainInd;
net.divideParam.valInd = valInd;
net.divideParam.testInd = testInd;

net.trainParam.lr = 1e-0;
net.trainParam.epochs =100;

net = configure(net,totalInput',totalOutput') ;

net = train(net,totalInput',totalOutput') ; 
outputs = net(test') ;

plotconfusion(testtargets',outputs);
output = zeros(size(outputs,2),8);
for i=1:size(outputs,2)
    output(i,:) = outputs(:,i) == max(outputs(:,i));
end
accuracy = sum(sum(validtargets == output))*100/(112*8)   
toc;