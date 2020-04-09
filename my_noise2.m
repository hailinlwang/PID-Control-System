% Generates a random noise value for encoder readings
% Noise is from normal distribution with mean of 1 and variance of 0.0001
% Approx <4% error in encoders
function x = my_noise2()
     x=1+sqrt(0.008)*randn;
      
end

