base of the code is from Udacity project Github, https://github.com/udacity/CarND-Path-Planning-Project
Used spline method provided by the project walkthrough.

Here is the logic how to drive the car:
1. drive car
2. check cars on each lane
3. sort cars on each lane
4. check cars just in front of me and just behind me on each lane
5. if the leading car on my lane is within 60 meter and slower than mine, prepare to change lane
6. if the leading car on my lane is within 30 meter and slower than mine, slow down the my car's speed to the leading car's speed. Acceleration is calculated using the relative speed of the leading car and my car and the distance. Keep checking if changing lane is possible
7. if leading car speed is higher than mine, accelerate car but not exceeding speed limit
8. if 'change flag' is not activated, go to #1
9. if flag is 'change lane'
10.   using the sensed speed ond position f front/behind car, predict the cars' position during/after changing lane
11.   if the predicted car's position is within 30meter, give the lane maximum cost
12.   else calculate cost using sigmoid function of predicted position
13.        the cost should encourage keeping the same lane giving reward of 30% reduction of the cost
14.        the cost should encourage change one lane giving reward of 29% reduction of the cost
15.        this means same_lane_cost = original_cost*0.7, nearst_lane_cost = original_cost*0.71
14.        the cost should encourage left side lane instead of right side lane, 10% reward for left side lane change
15.   if calcualted lane cost suggest change lane more than one lane
16.        check if middle lane is safe as an interim target lane, the cost should be 0.5*0.7. 
17.                 0.5 means none of the car in the middle lane is closer than 30meter  
18.        if middle lane is safe as an interim target or actually middle lane cost is lower than current lane, set the target lane to the middle lane 
19. go to #1

Future imporvements:
1. Use more accurate prediction of each lane car if they aggressively change lane near my car
2. Used speed instead of s_dot because usually vx is significant lower than vy but later s_dot can be used for more accurate model
