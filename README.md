# Robot Localization Project 
### Sam Wisnoski, Satchel Schiavo

## Project Goals:      
### Introduction:  
Our project goal was for a robot to be able to identify it's location and orientation given data from some LiDAR scans and a map of it's world frame. We used a particle filter to generate particles with a given pose and could then evaluate them to help create new particles until they converged onto a localization which represented our best pose estimate. We did this by mapping the closest LiDAR scans to each particle and comparing the scans to the map, and repopulated using a mix of kept 'high fitness' particles, a Gaussian distribution of particles centered around based off those same kept particles, and new, random particles, all of which were then given some random 'noise' or slight variation, and normalized. Our final filter could converge fairly quickly and would consistantly match the location of the Neato, and would not fall into local minimums.

Two examples of our particle filter working can be found here: 
* Path One: https://www.youtube.com/watch?v=UJY09rE9wLA
* Path Two: https://www.youtube.com/watch?v=vSg7XNA64JA

### Individual Learning Goals:   

#### Satchel's learning goals: 
My main learning goals for learning in depth how the Particle Filter math worked, diagnosing errors in the filter and understanding how playing with terms in our equation impacted both the accuracy of our converged localization as well as create bugs, and finally as a stretch learning goal understand and attempt to implement the Kalman filter. The first two were derived from mostly being interested in how different algorithms/ numerical methods are used in robotics and as it built nicely upon previoulsy projects I had done. I spent a lot of time working onthe pseudocode of the filter, and divided work with Sam to write the code for the evaluation and robot localization functions. My final learning goal of working with the Kalman filter after the lecture in class because it seemed like a real world extension of the particle filter. This proved difficult to implement but I spent a lot of time looking at papers and understanding the math behind it.

#### Sam's learning goals:
For this project, I wanted to focus on two main goals: 1) understanding a particle filter mathematically; and 2) implimenting code/functions in a new & unfamiliar codebase. The first goal was directly achieved in the early stages of the 
project as Satchel and I worked together to understand the various components of the particle filter. Before we even touched the code, we sat down and went function by function to gain a full understanding of how our filter worked. My second goal was a bit harder to reach; although we did impliment functions in the existing skeleton code, I felt it wasn't pretty. As we wrote each function, I had to constantly read through the helper functions, trying to use them as my guide to what angle we should approach our own implementation from. In the end, I ended up going through and commenting through the entire codebase in order to truly understand what was happening programatically. It was only after this that I felt I truly reached my second goal. Also, as Satchel mentioned, we shared work evenly across the project, from implementation, to testing, to write-up. 


## Methodology Overview:   

### Main loop: 
Our methodology can be demonstrated as a looping function that repeats five steps (plus one bonus, unlooped initialization step). These steps are described in detail below.    
![Loop of our robot localization functionality](media/localization_diagram.jpg)


#### Step 1: Initialization + Movement
In the first step of the main loop, we initialize our particle cloud. Each particle represents a possible robot pose (x,y,θ) within the world map. The initialization function generates a uniform spread of particles across the free space in the map, assigning each a random orientation and equal weight. This ensures the filter starts unbiased — the robot could, in theory, be anywhere in the environment. The number of particles was tuned so that there were enough to cover the map while still running efficiently. This initial gaussian spread gives our filter the necessary diversity to begin converging once LiDAR data starts coming in. Once we start recieving LiDAR data, we check every time we recieve data to check if the robot has moved far enough to update our pose estimate. This is done simply by using a linear and angular distance threshold. 

#### Step 2: Updating Particle Positions
The second step applies the robot’s movement model to predict how the particles should move between pose updates. We use odometry data (change in position and angle) to shift each particle according to the robot’s estimated movement. By iterating through the particles, each particle’s pose is updated. We then add small amounts of Gaussian noise to all three components to simulate real-world uncertainty and prevent the particles from clustering too tightly around incorrect positions. This step essentially predicts where the robot could now be based on motion, before correcting that guess using LiDAR data in the next steps.


#### Step 3: Weighting the Particles 
Our weighting function is made from mapping the closest laser scans onto each particle, and then comparing that to the map to check for accuracy. We do this by evaluating an error by looping through every one of our new particles in the particle and every one of our finite lidar scan. We then map the beam's endpoints to each particle and compare that to world map's obstacles. By returning that distance, we punish inaccurate scans. We actually square this distance as a design choice to even further filter out far off points. Finally, sum all of the errors of each laser scan to a net error per particle, then use a Gaussian probabibility model to transform the error into a weight for the particle, with higher weights corresponding to more likley poses in the particle cloud.

#### Step 4: Updating the Robots Pose
Step 4 involves calculating a new, summarized pose from the particle cloud's data. We begin by iterating through every particle in the cloud and summing the x and y position multiplied by the particle's weight to create an entire new x and y point. To find the new theta, we cannot simply sum the angles, but we can sum the sins and cosines first by splitting every particle's theta into the two values, then multiplying by the weights again, then recombining for theta. One last little change is we must transform the angle into quaternion, so the Pose() function can read it. Finally we use transform_helper to verify it's a real pose.

#### Step 5: Resampling and Normalization
After calculating the new pose estimate, we repopulate the particle cloud based on each particle’s weight. The resampling process uses a probabilistic approach where higher-weighted particles are more likely to be selected for the next generation. We split resampling into three categories:
* 60% kept particles: randomly chosen with a bias toward higher weights to preserve good estimates.
* 30% random particles: uniformly sampled across the map to maintain global exploration and prevent local minima.
* 10% Gaussian particles: generated by perturbing high-weight particles within a local radius, refining accuracy around strong candidates.
Finally, we apply small random noise to each particle and normalize all weights so they sum to one. This maintains numerical stability and ensures that the new particle set remains a proper probability distribution over all possible poses.


### Final Result:   
The individual steps can be seen in this gif below, where the particle filter is able to converge within the first few updates.   
<p align="center">
  <img src="media/convergence_path_1.gif" alt="Gif of our particle filter converging" width="800">
</p>

## Design Decisions:   

### Particle Resampling
One of our design decisions was in our resampling function. We split all new particles into three distinct groups - first 60% are 'kept' particles, or randomly chosen from the previously iteration of particles, with higher weighted particles being more likely to be chosen. We chose to implement this instead of simply keeping the 60% highest weighted particles to ensure some randomness in the localization - keeping the exact same 60% would be susceptiple to local minimums, where a particle could fall into a pose far closer to being correct than any of the surrounding poses, and would remain there throughout multiple iterations. To aid this, another 30% of resampled particles are completely random - this was done to help particles escape the local minumum, but also speed up exploration of far-off map areas. The final 10% were found from a Gaussian distribution of the kept particles, which conceptually works as a middle ground between the other two groups. We begin by determing a subsection of our kept particles, then randomly picking a point inside a radius centered around each kept particle. This helps the particle explore multiple close guesses, espieically prevneting it from getting trapped in areas with lots of local minimums, which will naturally increase as it converges closer and closer to the actual map space.

### Weighting Function
Designing the weighting function was one of our most important and challenging tasks. The weight determines how close each particle’s pose is to the actual robot using the most recent LiDAR scan. Initially, we tried using an inverse function, which worked initially, but errored out once we started moving through the hallway. We also noticed that particles with fewer valid LiDAR hits were incorrectly favored, since the total error was divided by fewer points. After several rounds of debugging and testing, we redesigned our weighting function around a Gaussian likelihood model, where particles with smaller average errors receive exponentially higher weights:
```particle.w = (math.exp(- (error_sum / count) / 0.5))**2```. This exponential decay penalizes large scan mismatches while heavily rewarding those that align closely with obstacles on the map. The squaring term further sharpens the contrast between good and bad particles, improving convergence speed. With this change, our particle filter consistently localized accurately and avoided drifting or locking into incorrect positions.

## Challenges:   
We had many small challenges for this project, (syntax, formatting, variable types, typos, etc), but the two big challenges are mirrored in our design decisions. First, we spent a lot of testing our resampling function to see what worked best. We had started out with just keeping some particles EXACTLY as they were (no noise) and resampling just based on gaussian, but Satchel quickly realized this design was prone to local minima. We then decided to add a large sampling of randomized particles to help with this problem, as well as add noise to every particle each time we resampled.

Our second major challenge was finding a good weighting function. Initially, we used an inverse function (1/x) to calculate weights, meaning that higher error values resulted in higher weights—exactly the opposite of what we wanted. We also summed the error based on the number of valid scans, which unintentionally gave LiDAR data with fewer scans higher weights. In reality, we wanted particles with more valid scan information to receive higher weights.
We resolved both issues by redefining our weighting function using an exponential Gaussian model: `particle.w = (math.exp(- (error_sum / count) / 0.5))**2`. This change made our particle filter perform nearly perfectly, converging correctly in every run.

## Potential Improvements:   
As a stretch goal for this project, we wanted to implement the Kalman filter and test the difference between the two. Would one converge faster, would it be more prone to errors? In the end, we found implementing a homemade Kalman filter to be too tricky, but would love to try again if given more time for the project. We would also want to improve convergence as the robot is turning, as we have the found the most errors with this, although this is likley due to a lack of data points.

## Conclusion:  
#### How It Went:
Overall, our particle filter performed well once the weighting function and resampling logic were tuned. The localization was both stable and responsive, converging to the correct pose within just a few iterations even from a uniform start. The robot tracked accurately through the environment, maintaining alignment with the real-world map. In our test videos, the filter recovered quickly from motion and consistently reflected the robot’s position, even in complex parts of the map.

#### Lessons Learned: 
This project gave us a much deeper understanding of probabilistic localization and how theory translates into code. We learned how subtle design choices, such as how to calculate weights or add noise, can drastically affect convergence and robustness. Debugging numerical issues also forced us to better understand the math behind the particle filter, not just the code. Perhaps most importantly, we learned the value of balancing randomness and structure: too much exploration leads to noise, but too little causes stagnation. With more time, we would explore adaptive resampling and noise scaling, as well as integrating a Kalman filter to compare deterministic and probabilistic localization methods.
