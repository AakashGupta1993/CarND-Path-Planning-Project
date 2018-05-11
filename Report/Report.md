
# Term 3 - Project 1
## Path Planning Project

## Goal

The goal of this project is to drive around the highway of 6946m (4.31 miles)in about 5 minutes.

We have to provides waypoints in the global co-ordinate system such that while moving in the virtual track there is no collision with other vehicles. Further we should not exceed the speed limit of 50 miles per hour. Apart from this our car should be able to overtake without colliding with other cars. While doing all this comfort of the passenger should be taken care of by not allowing jerky movement of the car.

## Path Generation
There are two approaches to this project :

    1) Defie a cost function and change lanes according to that cost function. 
    2) Carefully choose distance to change lanes.
I have followed the second approach as I found it to be simpler and it also aligns with the project Q & A video.

### Basic Intuition
Lane is changes only if there is a slow moving car in the current lane. The safe distance to check this is choosen as 30m in the front. But we cannot change the lane only if the car in front is moving slow. We need to check :

1) There is no other car in the other lane to which our car can collide on shifting.

2) We need to check if there is any other car in the other lane in front.

3) If car is there in the front in the lane we are shifting, our car should not collide with it.

4) We needdc to check if a car is coming from back in the other lane.

5) If car is coming from back side, our car should not shift. It should let it pass or let it slow down.

6) If we can shift lanes to right or left.  Car should not change lane to left if it is already in the leftmost lane. Similarly it should not move further towards right if it is already in the rightmost lane.

After validating all the above points the car turns towards the lane where it can go maintaining the max speed allowed in the highway.

### Flow of code :
    1) Store cars based on lanes.
    2) If there is a slow car in our lane, check other lanes for shifting.
    3) Find appropriate lane for shifting.
    4) Use spline library to generate smooth path and change the lanes.
    5) Change the path.
