//Helper file for path planning
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

//variable for cars on lane. This variable includes the car's s-position, d-position, and speed
struct Car_on_lane
{
    double s;
    double d;
    double speed;
};

//sort cars on each lane by s-position
bool sortByS(const Car_on_lane &lhs, const Car_on_lane &rhs) { return lhs.s < rhs.s; }

//getting lane information from given d-position
//if d-value is out of range, return unreasonable lane number so it can be discarded
int getLane(double d)
{
    if (d >= 0 && d < 4) return 0;
    if (d >= 4 && d < 8) return 1;
    if (d >= 8 && d < 12) return 2;
    return 100;
}

//Each lane info includes cars' information that are just in front of my car and just behind my car
struct Lane_info
{
    Car_on_lane car_behind;
    Car_on_lane car_front;
};

//used sigmoid funtioin to calculate cost as this fucntions returns 0 to 1 depending on the given parameter
//drastic cost change near the car but almost flat change for cars very far from my car
double sigmoid(double x)
{
    return 1/(1+exp(-x));
}

//calculate cost using lane info and my car info
float getCost(int target_lane, Lane_info lane, int car_lane, double car_s, double car_speed )
{

    double distance_front = lane.car_front.s - car_s;
    double rel_speed_front = lane.car_front.speed - car_speed;
    double predicted_distance_front = distance_front + rel_speed_front*0.02*10; 
    if (abs(predicted_distance_front) < 30) return 1.0; //if leading car is going to be too close after changing lane, cost is maximum

    double distance_behind = car_s - lane.car_behind.s;
    double rel_speed_behind = car_speed - lane.car_behind.speed;
    double predicted_distance_behind = distance_behind + rel_speed_behind*0.02*10; 
    if (abs(predicted_distance_behind) < 30) return 1.0; //if car behind is going to be too close after changing lane, cost is maximum

    //calculate cost with near car's position after changing lane
    return (sigmoid((30 - predicted_distance_front)/10.0) + sigmoid((30 - predicted_distance_behind)/10.0))/2.0;

}

//using the cost function, make a decisioin for lane change
int getTargetLane(double car_s, double car_speed, int car_lane, vector<Lane_info> near_car_on_lane)
{
    vector<double> lane_cost(3);
    for (int i = 0; i <= 2; i++)
    {
        
        lane_cost[i] = getCost(i, near_car_on_lane[i], car_lane, car_s, car_speed)
        //reward for keeping lane = cost*0.7, reward for changing lane to adjacnet lane = cost*0.75, no reward for changing lane to far side
        //encouraging either staying on lame lane or changing lane to left side
                        *(1-(i==car_lane)*0.3)*(1-(abs(i-car_lane)==1)*0.29)*(1-(i <= car_lane)*0.1); 
        cout << "lane " << i << " cost = " << lane_cost[i] << endl;
    }

    //set the target lane to minimum cost lane
    int target_lane = car_lane;
    for (int i = 0; i <= 2; i++)
    {
        if (lane_cost[i] < lane_cost[target_lane]) target_lane = i;
    }

    //if far side cost is minimum and risk is small or middle lane cost is smaller than current lane cost then move to middle lane
    if (abs(target_lane - car_lane) == 2) 
        if ((lane_cost[1] < 0.5*0.75) || (lane_cost[1] < lane_cost[car_lane])) target_lane = 1; 
        else target_lane = car_lane;

    return target_lane;
}
