// A sample code for assignment #1 using C++ class

#include <ros/ros.h>
#include <iostream>     			// to use: std::cout, std::cin, etc
#include <cmath>					// to use: atan2, sqrt, pow, etc
#include <iomanip>     				// to use: setprecision()
#include <geometry_msgs/Twist.h> 
#include <turtlesim/Pose.h>  


using namespace std;

/*/////////////////////////////////////////////////////////////*/
/*/////////////////////////////////////////////////////////////*/
/*/////////////////////////////////////////////////////////////*/
// THIS IS THE TURTLE CLASS

class TurtleClass
{
	public:
		/*/////////////////////////////////////////////////////////////*/
		/*/////////////////////////////////////////////////////////////*/			
		// constructor prototype
		TurtleClass(ros::NodeHandle& n);

		/*/////////////////////////////////////////////////////////////*/
		/*/////////////////////////////////////////////////////////////*/			
		// DESTRUCTOR
		~TurtleClass()
		{
		}

	/*/////////////////////////////////////////////////////////////*/
	/*/////////////////////////////////////////////////////////////*/	
	// THESE ARE THE PRIVATE VARIABLES
	private:
        ros::Subscriber sub;				// create subscriber object;
        ros::Publisher pub;					// create publisher object;
		turtlesim::Pose cp;					// current position
		turtlesim::Pose dp;					// desired position	
		turtlesim::Pose err;				// error (Pose: x,y,z rot_x, rot_y,rot_z)
		int counter;						// counter
		static const float tol = 0.1;		// error tolerance
        static const float p_gain = 0.5;	// proportional gain (default: 0.5)
        float err_euc;                      // error (Euclidean distance between current and goal position)	

		// Functions prototypes...
		void TurtleCallback(const turtlesim::Pose& msg);
		void TurtleGetVal(void);
		void TurtleError(void);
		void TurtleMove(void);

};
/*/////////////////////////////////////////////////////////////*/
/*/////////////////////////////////////////////////////////////*/
// CONSTRUCTOR
TurtleClass::TurtleClass(ros::NodeHandle& n)
{
    // create publisher object, will be used to send to turtle1/cmd_vel topic with queue size of 10
    pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);

    // create subscriber object based on TurtleClass::TurtleCallback function
	sub = n.subscribe("turtle1/pose",10,&TurtleClass::TurtleCallback,this); 

	// define loop rate to be 10Hz
	ros::Rate loop_rate(10);
	counter = 0;
	// runs this loop until there is terminate singal
	while(ros::ok()) 
	{	
		counter++;          // increase counter
		ros::spinOnce();	// call callback functions

		// delay in running callback functions (roughly taks 1-2 counters)
		if (counter > 3)
		{
			// compute error
			TurtleError();

			// when error to the goal becomes small enough --OR-- if this is the 1st time, 
			if (abs(err_euc) < tol || counter == 4)
			{
		        // display some messeages to screen and asks user for the next inputs...

				TurtleGetVal();
			}
			// otherwise keep moving the turtle
			else
			{
				TurtleMove();
			}
		}
		// this is necessary to prevent excess cpu usage (this is while loop)
		loop_rate.sleep();
	}																																		
}

/*/////////////////////////////////////////////////////////////*/
/*/////////////////////////////////////////////////////////////*/			
// CALLBACK FUNCTION
void TurtleClass::TurtleCallback(const turtlesim::Pose& msg)
{
	// setting current position/orientation values with the messages received 
	// from the turtlesim/Pose topic
    cp.x = msg.x;               // current position (x)
    cp.y = msg.y;               // current position (y)
    cp.theta = msg.theta;       // current orientation (theta)
}

/*/////////////////////////////////////////////////////////////*/
/*/////////////////////////////////////////////////////////////*/	
// DISPLAY POSE, GET USER INPUT
void TurtleClass::TurtleGetVal(void)
{
    /*/////////////////////////////////////////////////////////////*/
    /*/////////////////////////////////////////////////////////////*/
    // display current status (Pose, distance to goal) of the turtle

	cout << endl;
	cout << "---------------------------------------------" << endl;
	cout << "Current pose [xc,yc,th]=[" << setprecision(3) << cp.x << "," 
		<< setprecision(3) << cp.y << "," << setprecision(3) << cp.theta << "]" << endl;
	cout << "Previous goal [xg,yg]=[" << setprecision(3) << dp.x << "," 
		<< setprecision(3) << dp.y << "]" << endl;
	cout << "Error ||xg-xc,yg-yc||=" << setprecision(3) << err_euc 
		<< " (err tol=" << tol << ")" << endl;
	cout << "---------------------------------------------" << endl;			
	cout << endl;	
	cout << "Enter x y coordindates of the goal (e.g., 10 10 means x=10,y=10)" << endl;
	cout << "or CTRL+C to exit" << endl;
    cout << endl;

    /*/////////////////////////////////////////////////////////////*/
    /*/////////////////////////////////////////////////////////////*/


	// get user input...
    cin >> dp.x >> dp.y;                // desires position (x y)
}

/*/////////////////////////////////////////////////////////////*/
/*/////////////////////////////////////////////////////////////*/	
// COMPUTE POSE ERRORS
void TurtleClass::TurtleError(void)
{
    // correct way to compute angle (using atan2(y,x))
	dp.theta = atan2(dp.y-cp.y,dp.x-cp.x);
	
	// angle difference between turtle's orientation and angle to goal position 				
	err.theta = atan2(sin(dp.theta - cp.theta),cos(dp.theta - cp.theta));

	// position error between current and goal position
    err.x = dp.x - cp.x;            // error in x
    err.y = dp.y - cp.y;            // error in y
	// euclidean distance from current to goal position
    err_euc =  sqrt(pow(err.x,2) + pow(err.y,2));	// error sqrt(x^2 + y^2)
}

/*/////////////////////////////////////////////////////////////*/
/*/////////////////////////////////////////////////////////////*/	
// GENERATE CONTROL, AND SEND PUBLISH IT TO TURTLE
void TurtleClass::TurtleMove(void)
{
    geometry_msgs::Twist vel;               // create publisher object vel with messeage type geometry_msgs::Twist

    // proportional controller
    vel.linear.x = p_gain * err_euc;        // linear velocity (x)
    vel.angular.z = p_gain * err.theta;     // angular velocity (z)
	
    // publish data	to 'vel'
	pub.publish(vel);
	
	// display distance to goal...
	cout << left << setw(20) << "Turtle is moving! " << setw(15) << "Distance to goal=" 
		<< setw(20) << setprecision(3) << err_euc << endl;
}	


/*/////////////////////////////////////////////////////////////*/
/*/////////////////////////////////////////////////////////////*/
/*/////////////////////////////////////////////////////////////*/
// MAIN FUNCTION

int main(int argc, char **argv)
{
	// create node 'sample_node' (ros::init_options::NoSigintHandler works file for no reason, 
	// I need to figure out, without it if you try Ctrl+C at cin, it won't work...) 
	ros::init(argc, argv,"sample_node",ros::init_options::NoSigintHandler);  
    ros::NodeHandle n;					// create node handle
    // create a TurtleClass
	TurtleClass turtle(n);

	return 0;
}





