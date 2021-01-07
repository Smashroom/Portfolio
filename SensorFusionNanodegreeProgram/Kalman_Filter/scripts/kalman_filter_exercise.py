from math import *

def f(mu,sig2,x):
    return exp(-0.5*(pow(x-mu,2)/sig2))/sqrt(2*pi*sig2)

def update(mu, sig2, nu, ro2):
    # Gauss multiplication
    # 1st Gaussian : (mu, sig2)
    # 2nd Gaussian : (nu, ro2)
    print(mu, nu, sig2, 1/float(ro2))
    mu_new = (ro2*mu + sig2*nu)/float(sig2+ro2)
    sig2_new = 1/(1/float(sig2) + 1/float(ro2))
    return mu_new, sig2_new
    
def predict(mu, sig2, nu, ro2):
    # Gauss addition
    mu_new = mu + nu 
    sig2_new = sig2 + ro2
    return mu_new, sig2_new
    
#print(f(10., 4., 8.))
#print(gauss_multiplier(10, 13, 8, 2))

# 1D Kalman Filter

measurements = [5., 6., 7., 8., 9., 10.] 
motion = [1., 1., 2., 1., 1.]
measurements_var = 4
motion_var = 2
mu = 0
var = 1000

for i in range(0,5):
    [mu,var] = update(mu, var, measurements[i], measurements_var )
    [mu,var] = predict(mu, var, motion[i], motion_var)
    
    print("Kalman filter step:",i,"mu:",mu,"var:",var)