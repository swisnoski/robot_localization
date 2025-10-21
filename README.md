# Robot Localization Project 
### Sam Wisnoski, Satchel Schiavo

Our writeup should address the following questions:   
1. What was the goal of your project?
2. How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
3. Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
4. What if any challenges did you face along the way?
5. What would you do to improve your project if you had more time?
6. Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.


We can answer these questions with the following sections: 

## Project Goal:      
#### Introduction:  
Give a brief introduction of our problem, 

Our project goal was for a robot to be able to identify it's location and orientation given data from some LiDAR scans and a map of it's world frame. We used a particle filter to generate particles with a given pose and could then evaluate them to help create new particles until they converged onto a localization which represented our best particle guess. We did this by mapping the closest LiDAR scans to each particle and comparing the scans to the map, and repopulated using a mix of kept 'high fitness' particles, a Gaussian distribution of particles centered around based off those same kept particles, and new, random particles, all of which were then given some random 'noise' or slight variation, and normalized. Our final filter could converge fairly quickly and would consistantly match the location of the Neato, and would not fall into local minimums.


#### Individual Learning Goals:   
Write two-three learning goals per person, explain how we divided work to meet those goals 

Satchel's learning goals: My main learning goals for learning in depth how the Particle Filter math worked, diagnosing errors in the filter and understanding how playing with terms in our equation impacted both the accuracy of our converged localization as well as create bugs, and finally as a stretch learning goal understand and attempt to implement the Kalman filter. The first two were derived from mostly being interested in how different algorithms/ numerical methods are used in robotics and as it built nicely upon previoulsy projects I had done. I spent a lot of time working onthe pseudocode of the filter, and divided work with Sam to write the code for the evaluation and robot localization functions. My final learning goal of working with the Kalman filter after the lecture in class because it seemed like a real world extension of the particle filter. This proved difficult to implement but I spent a lot of time looking at papers and understanding the math behind it.

Sam's learning goals:



## Methodology Overview:   
Explain methodology. Want to provide a lot of diagrams, gifs, videos etc here
Basically go through the main loop and describe what happens to our particles as we drive around (initialization, updating, weighting, pose estimation, redistributing) 
Overview the algorithms/equations. 

## Design Decisions:   
Will fill out more of this after we complete the project. Likely, will be about how we initialize/weight/redistribute the particles. 
Why gaussian? Do we reduce the number of particles over time? Satchel wanted to try out a different localization method as well, this 
would be a good place for that. 

## Challenges:   
We are great. We had no challenges whatsoever with this project. 

## Potential Improvements:   
TBD. Maybe that other localization method? Maybe testing it out on the actual robot? 

## Conclusion:  
#### How It Went: 
TBD. 

#### Lessons Learned:   
TBD. 
