#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <sstream>
#include <linux_hardware/LaptopChargeStatus.h>

#include <ctime>

#define SIMULATION_MODE 1
class motivational_component
{
public:

  motivational_component()
  {
    ROS_INFO("BASIC constructor INIT ->>>>>");
  }

  
  motivational_component(ros::NodeHandle node):
  n(node)
  {
    
    ROS_INFO("INIT Constructor ->>>>>");
    
    //This is easily adaptable to a topic callback
    robot_mode(SERVICE_MODE);

    ///*************************************************************************
    /// Publishers START *******************************************************
    ///*************************************************************************

    /// Motivational D-variables ==============================================
    //  Motivational rest: it depends of activity cycles, fatigue (battery), and user cycles
    p_dvar_rest_level = n.advertise<std_msgs::Int32>("dvar_rest_level", 5);

    //  Motivational Fear: Two kinds, lost (without localization), new people
    p_dvar_fear_lost_level = n.advertise<std_msgs::Int32>("dvar_fear_lost_level", 5);
    p_dvar_fear_humans_level = n.advertise<std_msgs::Int32>("dvar_fear_humans_level", 5);


    //  Motivational Comfort:
    p_dvar_pain_status = n.advertise<std_msgs::Int32>("dvar_pain_level", 5);


    /// Motivational B-variables ==============================================
    //  Motivational Comfort:
    p_bvar_comfort_status = n.advertise<std_msgs::Int32>("bvar_comfort_level", 5);

    // Motivational curi0sity
    p_bvar_curiosity = n.advertise<std_msgs::Int32>("bvar_curiosity_level", 5);

    // Motivational frustration
    p_bvar_frustration = n.advertise<std_msgs::Int32>("bvar_frustration_level", 5);


    // Internal variables =====================================================
    p_laptop_fatigue_level = n.advertise<std_msgs::Int32>("computer_fatigue_level", 5);
    p_robot_fatigue_level = n.advertise<std_msgs::Int32>("robot_fatigue_level", 5);
    p_day_cicle_fatigue_level = n.advertise<std_msgs::Int32>("day_cicle_fatigue_level", 5);

    p_task_fatigue_level = n.advertise<std_msgs::Int32>("task_fatigue_level", 5);
    p_energy_fatigue_level = n.advertise<std_msgs::Int32>("energy_fatigue_level", 5);

    p_task_frustration_level = n.advertise<std_msgs::Int32>("task_frustration_level", 5);
    //p_timer_frustration_level = n.advertise<std_msgs::Int32>("timer_frustration_level", 5);

    p_bumper_total_hits = n.advertise<std_msgs::Int32>("bumper_hits", 5);

    p_motivational_status = n.advertise<std_msgs::String>("status", 5);

    /// Publishers END *********************************************************

    ///*************************************************************************
    /// Subscribers START ******************************************************
    ///*************************************************************************

    // Activations
    s_task_fatigue_activation 		= n.subscribe("/motivational_component/task_fatigue_activation", 1000, &motivational_component::fatigue_task_activation_callback, this);
    s_task_curiosity_activation 	= n.subscribe("/motivational_component/task_curiosity_activation", 1000, &motivational_component::curiosity_task_activation_callback, this);
    s_task_frustration_activation 	= n.subscribe("/motivational_component/task_frustration_activation", 1000, &motivational_component::frustration_task_activation_callback, this);


    // Reads
    s_battery_status 		= n.subscribe("laptop_charge", 1000, &motivational_component::laptop_battery_status_callback, this);
    s_frustration_level 	= n.subscribe("/motivational_component/task_frustration_update", 1000, &motivational_component::frustration_update_callback, this);

    bumper_Subs = n.subscribe("/bumper", 1000, &motivational_component::updateBumperCounter_callback, this);

    update = n.subscribe("/rate", 1000, &motivational_component::rate_update_Callback, this);
    

    /// Subscribers END *************************************************************************

    /// Timers END *************************************************************************
    /**
      * Timers allow you to get a callback at a specified rate.
      * Here we create two timers at different rates:
      * timer1
      */
    timer_motivational = n.createTimer(ros::Duration(1.0),  &motivational_component::timer_motivational_callback, this);

    timer_task_fatigue = n.createTimer(ros::Duration(10.0), &motivational_component::fatigue_task_timer_callback, this);
    timer_task_fatigue.stop();

    timer_curiosity = n.createTimer(ros::Duration(1.0), &motivational_component::curiosity_timer_callback, this);
    timer_curiosity.stop();

    timer_human_rutine_cycle = n.createTimer(ros::Duration(60.0), &motivational_component::fatigue_human_rutine_timer_callback, this);
    timer_human_rutine_cycle.start();

    timer_overall_fatigue = n.createTimer(ros::Duration(20.0), &motivational_component::fatigue_overall_timer_callback, this);
    timer_overall_fatigue.start();


    timer_frustration = n.createTimer(ros::Duration(5.0), &motivational_component::frustration_timer_callback, this);
    timer_frustration.stop();


    timer_pain = n.createTimer(ros::Duration(15.0), &motivational_component::pain_timer_callback, this);
    timer_pain.start();


    if( SIMULATION_MODE )
    {
		timer_robot_batt_simulation = n.createTimer(ros::Duration(20.0), &motivational_component::robot_batt_simulation_timer_callback, this);
		timer_robot_batt_simulation.start();

		timer_laptop_batt_simulation = n.createTimer(ros::Duration(60.0), &motivational_component::laptop_batt_simulation_timer_callback, this);
		timer_laptop_batt_simulation.start();
    }

    v_fatigue_variables_normalization = 3;
    //    timer_human_rutine_cycle.stop();

    v_time_now = time(0);
    now = localtime( & v_time_now );


    ROS_INFO(" %s", asctime(now));

    //    ros::Time begin = ros::Time::now();
    //    boost::posix_time thistime = from_time_t(begin);
    //    std::string to_simple_string(thistime);
    //    ROS_INFO(" Time %i", begin);

  }


  
void robot_mode(const int mode)
{
    v_control_loop_rate = 1;
    robot_bumper         = 0;
    v_day_cycle_fatigue = 0;
    v_task_fatigue      = 0;
    v_overall_fatigue   = 0;
    v_curiosity_level   = 0;

    robot_safe_status_level     = 10;
    v_tired_level_status        = 0;
    v_laptop_energy_fatigue     = 100;
    v_robot_energy_fatigue       = 100;

    robot_bumper = 1;
    robot_pain_status_level = 0;
    bumper_hit_simulation = 0;

      
    if(mode == SERVICE_MODE)
    {
        v_threshold_pain                = 3;
        v_threshold_frustration_time    = 90;
        v_threshold_max_curiosity_level = 90;
        v_threshold_max_bumper_hits     = 30 ;
        
        //These values are daytime hours
        v_threshold_morning_start       = 8;
        v_threshold_launch_start        = 13;
        v_threshold_siesta_start        = 16;
        v_threshold_siesta_end          = 17;
        v_threshold_afternoon_start     = 17;
        v_threshold_dinner_start        = 21;
        
        
    }
    else if (mode == COMPANION_MODE)
    {
        v_threshold_pain = 5;
        v_threshold_frustration_time = 90;
        v_threshold_max_curiosity_level = 90;
        v_threshold_max_bumper_hits = 30;
        
        //These values are daytime hours
        v_threshold_morning_start       = 8;
        v_threshold_launch_start        = 13;
        v_threshold_siesta_start        = 16;
        v_threshold_siesta_end          = 17;
        v_threshold_afternoon_start     = 17;
        v_threshold_dinner_start        = 21;

    }
    else if (mode == ASSISTIVE_MODE)
    {
        v_threshold_pain = 10;
        v_threshold_frustration_time = 90;
        v_threshold_max_curiosity_level = 90;
        v_threshold_max_bumper_hits=30;
        
        //These values are daytime hours
        v_threshold_morning_start       = 8;
        v_threshold_launch_start        = 13;
        v_threshold_siesta_start        = 16;
        v_threshold_siesta_end          = 16;
        v_threshold_afternoon_start     = 17;
        v_threshold_dinner_start        = 21;

    }
  
}
  

//It is not possible to change the rate when started
//const std_msgs::Int32::ConstPtr& message
void
rate_update_Callback(const std_msgs::Int32::ConstPtr& v_rate)
{
  ROS_INFO("I receive a change rate message: [%d]", v_rate->data);
}


/*! Motivational subsystem initialization
*
*/
void
timer_motivational_callback(const ros::TimerEvent&)
{
	if(DEBUG)
		ROS_INFO("callback pain timer ");

	std_msgs::String msg;

	std::stringstream ss;
	ss << "System OK";
	msg.data = ss.str();

	p_motivational_status.publish(msg);

}



/*! Pain Level subsystem initialization
 *  S
*  0 : ok
*  10: Pain
*/
void
pain_timer_callback(const ros::TimerEvent&)
{
	if(DEBUG)
		ROS_INFO("callback_motivational_timer 1 triggered");


	if(SIMULATION_MODE)
	{
		if(++bumper_hit_simulation % 5 == 0)
			robot_bumper++;
	}

	if(robot_bumper % v_threshold_pain == 0)
	{
		ROS_INFO("Ouch, we hit something %d times. The pain value is increased %d,[%d] - %d, %d",v_threshold_pain,  robot_pain_status_level,robot_bumper, robot_bumper % v_threshold_pain, int(robot_bumper % v_threshold_pain));

		robot_pain_status_level++;
		t_pain_level.data = robot_pain_status_level;
		p_dvar_pain_status.publish(t_pain_level);

                //May we should decide to stop the robot here or maybe cry :D
		if(robot_bumper == v_threshold_max_bumper_hits)
                  robot_bumper = 1;

	}

	t_bumper_total_hits.data = robot_bumper;
	p_bumper_total_hits.publish(t_bumper_total_hits);

}


  

/**
* We have a counter for bumper information
* After 10 hits the warning increase
*
*/
void
updateBumperCounter_callback(const std_msgs::Int32::ConstPtr& v_rate)
{
	ROS_INFO("I receive a change rate message: [%d]", v_rate->data);


	robot_bumper++;

	if(robot_bumper < 0)
	{
		robot_safe_status_level++;

		ROS_INFO("Hemos chocado demasiadas veces diminuimos el valor safe %d", robot_safe_status_level);

		robot_bumper = 10;

	}

}



/** \fn void laptop_battery_status_callback(const std_msgs::Int32::ConstPtr& v_status)
  \brief TODO
  <node pkg="linux_hardware" type="laptop_battery.py" name="turtlebot_laptop_battery">

  \param v_status ROS value from callback
*/
void
laptop_battery_status_callback(const linux_hardware::LaptopChargeStatusConstPtr& v_status)
{

	v_laptop_energy_fatigue = 100 - v_status->percentage;

	t_laptop_fatigue_level_status.data = v_laptop_energy_fatigue;


	if(DEBUG_LAPTOP_BATT)
		ROS_INFO("Laptop Battery Status: [%d]", v_laptop_energy_fatigue);

	p_laptop_fatigue_level.publish(t_laptop_fatigue_level_status);

}



/** \fn void laptop_batt_simulation_timer_callback(const ros::TimerEvent&)
  \brief TODO
 
  \param v_status ROS timer callback
*/
void
laptop_batt_simulation_timer_callback(const ros::TimerEvent&)
{

	v_laptop_energy_fatigue --;

	t_laptop_fatigue_level_status.data = v_laptop_energy_fatigue;


	if(DEBUG_LAPTOP_BATT)
		ROS_INFO("Laptop Battery Status: [%d]", v_laptop_energy_fatigue);

	p_laptop_fatigue_level.publish(t_laptop_fatigue_level_status);

}


/** \fn void robot_battery_status_callback(const std_msgs::Int32::ConstPtr& v_status)
  \brief TODO
 
  \param v_status ROS value from callback
*/
void
robot_battery_status_callback(const linux_hardware::LaptopChargeStatusConstPtr& v_status)
{
	//int laptop_batt = v_status->percentage;
	v_robot_energy_fatigue = 100 - v_status->percentage;
	t_robot_fatigue_level_status.data = v_robot_energy_fatigue;

	if(DEBUG_LAPTOP_BATT)
		ROS_INFO("Robot Battery Status: [%d]", v_robot_energy_fatigue);


	p_robot_fatigue_level.publish(t_robot_fatigue_level_status);

}

/** \fn void updateBatt(const std_msgs::Int32::ConstPtr& v_status)
  \brief TODO

  \param  ROS value callback from timer
*/
void
robot_batt_simulation_timer_callback(const ros::TimerEvent&)
{
	//int laptop_batt = v_status->percentage;
	v_robot_energy_fatigue --;
	t_robot_fatigue_level_status.data = v_robot_energy_fatigue;

	if(DEBUG_LAPTOP_BATT)
		ROS_INFO("Robot Battery Status: [%d]", v_robot_energy_fatigue);


	p_robot_fatigue_level.publish(t_robot_fatigue_level_status);

}

/*! \fn void tired_activation_callback(const std_msgs::Int32::ConstPtr& v_rate)
    \brief Activate or deactivate the tired variable (0 or 1)
    \param v_status ROS value from callback
*/
void frustration_task_activation_callback(const std_msgs::Int32::ConstPtr& v_status)
{
  ROS_INFO("I receive a message of task_frustration  activation/deactivation [%d]", v_status->data);

  if( v_status->data == 1)
  {
		v_frustration_time = 0;
		timer_frustration.start();
  }
  else
  {
	  	timer_frustration.stop();
  }

}

/*! \fn void frustration_update_callback(const std_msgs::Int32::ConstPtr& v_rate)
    \brief Activate or deactivate the tired variable (0 or 1)
    \param v_status ROS value from callback
*/
void frustration_update_callback(const std_msgs::Int32::ConstPtr& v_status)
{
  ROS_INFO("I receive a message of frustration_update [%d]", v_status->data);

  v_task_frustation_level += v_status->data;
  if( v_task_frustation_level > 0)
  {
	  t_task_frustration_level.data = v_task_frustation_level;
  }

}


/*! Frustration
 *
 *   \param  ROS value callback from timer
 */
void
frustration_timer_callback(const ros::TimerEvent&)
{
	if(DEBUG)
		ROS_INFO("Callback Timer: ");

	v_frustration_time++;

	if (v_frustration_time > v_threshold_frustration_time )
			v_frustration_time = 80;


	t_time_frustration_level.data = v_frustration_time;
	p_bvar_frustration.publish(t_time_frustration_level);



}


/*! \fn void task_fatigue_activation_callback(const std_msgs::Int32::ConstPtr& v_rate)
    \brief Activate or deactivate the tired variable (0 or 1)
    \param v_status ROS value from callback
*/
void
fatigue_task_activation_callback(const std_msgs::Int32::ConstPtr& v_status)
{
  ROS_INFO("I receive a message of task_fatigue activation/deactivation [%d]", v_status->data);

  if( v_status->data == 1)
  {
	  v_task_fatigue = 0;
	  timer_task_fatigue.start();

  }
  else
  {
	  timer_task_fatigue.stop();
  }

}






void
fatigue_task_timer_callback(const ros::TimerEvent&)
{
	//It starts at 0

	v_task_fatigue++;
	v_day_cycle_fatigue++;
	t_task_fatigue_level.data = v_task_fatigue;
	p_task_fatigue_level.publish(t_task_fatigue_level);

	if(DEBUG)
		ROS_INFO("Callback: v_task_fatigue %d", v_task_fatigue);


}


void
fatigue_overall_timer_callback(const ros::TimerEvent&)
{
	//value to normalize the results
	//each variable go from 0 to 100;
	v_overall_fatigue = ( ( v_laptop_energy_fatigue + v_robot_energy_fatigue + (100-v_day_cycle_fatigue) ) / v_fatigue_variables_normalization ) ;

	t_rest_level.data = v_overall_fatigue;
	p_dvar_rest_level.publish(t_rest_level);

//	if(DEBUG)
	ROS_INFO("Callback: overall_fatigue %d. robot %d, laptop %d, cycle  %d, task %d , %d", v_overall_fatigue, v_robot_energy_fatigue, v_laptop_energy_fatigue, 100-v_day_cycle_fatigue, v_task_fatigue, ( ( v_laptop_energy_fatigue + v_robot_energy_fatigue + v_day_cycle_fatigue ) / v_fatigue_variables_normalization ));
}
  


void
fatigue_human_rutine_timer_callback(const ros::TimerEvent&)
{
	//	Human Routine
	//  From 00:00 to 9:00 sleep: fatigue decrease:
	//  From 9:00 to 15:00 fatigue increase: work
	//  From 15:00 to 17:00 fatigue decrease: siesta
	//  From 17:00 to 22:00 fatigue increase: work
	//  From 22:00 to 00:00 fatigue increase x2 : work


	//	std::cout << (now->tm_year + 1900) << '-'
	//		<< (now->tm_mon + 1) << '-'
	//		<<  now->tm_mday << '-'
	//		<< now->tm_hour << ':'
	//		<< now->tm_min << '-'
	//		<< std::endl;

	v_time_now = time(0);
	now = localtime( & v_time_now );

	v_hour_of_the_day = now->tm_hour;

	if (((v_hour_of_the_day > v_threshold_morning_start) && (v_hour_of_the_day < v_threshold_siesta_start )) || ((v_hour_of_the_day > v_threshold_siesta_end) && (v_hour_of_the_day < v_threshold_dinner_start )))
	{
//              ROS_INFO("Callback: My fatigue increase in x1 working hours");
		v_day_cycle_fatigue += 1;
	}
	else if((v_hour_of_the_day > v_threshold_dinner_start))
	{
//              ROS_INFO("Callback: My fatigue increase in x2 factor");
		v_day_cycle_fatigue += 2;
	}
	else if((v_hour_of_the_day > v_threshold_launch_start) && (v_hour_of_the_day < v_threshold_afternoon_start) )
	{
//		ROS_INFO("Callback: My fatigue increase in x3 factor");
		v_day_cycle_fatigue += 3;
	}
	else
	{
//		ROS_INFO("Callback: I should be sleeping, I feel tired so quickly");
		v_day_cycle_fatigue += 5;
	}
	t_day_cicle_fatigue.data = v_day_cycle_fatigue;
	p_day_cicle_fatigue_level.publish(t_day_cicle_fatigue);
}



/*! \fn void tired_activation_callback(const std_msgs::Int32::ConstPtr& v_rate)
    \brief Activate or deactivate the tired variable (0 or 1)
    \param v_status ROS value from callback
*/
void
curiosity_task_activation_callback(const std_msgs::Int32::ConstPtr& v_status)
{
	ROS_INFO("I receive a message of curiosity activation/deactivation [%d]", v_status->data);

	if( v_status->data == 1)
	{
		
		t_curiosity_level.data = 0;

		if(v_curiosity_level > 0)
			v_curiosity_level = v_curiosity_level - 10;

		timer_curiosity.start();
	}
	else
	{
		timer_curiosity.stop();
	}

}

void
curiosity_timer_callback(const ros::TimerEvent&)
{
	//It starts at 0

	v_curiosity_level++;
	t_curiosity_level.data = v_curiosity_level;

	p_bvar_curiosity.publish(t_curiosity_level);

	if(v_curiosity_level == 100)
	{
		ROS_INFO("Callback: Stopping curiosity timers");
		timer_curiosity.stop();
	}
	else if(v_curiosity_level > v_max_curiosity_level)
	{
		ROS_INFO("Callback: I want to pay attention whatever");
	}

}

private:
 
  int robot_bumper;
  int robot_pain_status_level;
  int bumper_hit_simulation;


  int robot_safe_status_level;


  int control_loop_rate;
  int v_control_loop_rate;
  int v_fatigue_variables_normalization;

  int v_tired_level_status;
  int v_min_tired_level;

  int v_curiosity_level;
  int v_max_curiosity_level;

  int v_overall_fatigue;
  int v_task_fatigue;

  int v_laptop_energy_fatigue;
  int v_robot_energy_fatigue;
  int v_day_cycle_fatigue;

  int v_hour_of_the_day;
  time_t v_time_now;
  struct tm * now;

  int v_frustration_time;
  int v_task_frustation_level;

  int v_threshold_pain;
  int v_threshold_frustration_time;
  int v_threshold_max_curiosity_level;
  int v_threshold_max_bumper_hits;
  
  
  int v_threshold_morning_start;
  int v_threshold_launch_start;
  //http://brocku.ca/sleeplab/articles/Napping%20in%20young%20and%20older%20adults.pdf
  int v_threshold_siesta_start;
  int v_threshold_siesta_end;
  int v_threshold_afternoon_start;
  int v_threshold_dinner_start;
  
  static const int DEBUG = 0;
  static const int DEBUG_LAPTOP_BATT = 1;


  static const int SERVICE_MODE = 0;
  static const int COMPANION_MODE = 1;
  static const int ASSISTIVE_MODE = 2;
  
  
  /**
  * Publishers
  *
  */
  ros::Publisher p_dvar_rest_level;

  ros::Publisher p_dvar_fear_lost_level;
  ros::Publisher p_dvar_fear_humans_level;

  ros::Publisher p_dvar_pain_status;
  ros::Publisher p_dvar_tired_level;

  ros::Publisher p_bvar_curiosity;
  ros::Publisher p_bvar_frustration;
  ros::Publisher p_bvar_comfort_status;

  ros::Publisher p_laptop_fatigue_level;
  ros::Publisher p_robot_fatigue_level;
  ros::Publisher p_day_cicle_fatigue_level;
  ros::Publisher p_task_fatigue_level;
  ros::Publisher p_energy_fatigue_level;

  ros::Publisher p_motivational_status;

  ros::Publisher p_task_frustration_level;
  ros::Publisher p_timer_frustration_level;

  ros::Publisher p_bumper_total_hits;

  /**
  * Subscribers
  *
  */
  ros::Subscriber bumper_Subs;
  ros::Subscriber update;
  ros::Subscriber s_tired_activation;
  ros::Subscriber s_curiosity_activation;
  ros::Subscriber s_battery_status;
  ros::Subscriber s_task_fatigue_activation;
  ros::Subscriber s_task_curiosity_activation;
  ros::Subscriber s_task_frustration_activation;
  ros::Subscriber s_frustration_level;
  
  /**
  * Timers
  *
  */
  ros::Timer timer_motivational;
  ros::Timer timer_curiosity;
  ros::Timer timer_task_fatigue;
  ros::Timer timer_human_rutine_cycle;
  ros::Timer timer_batt_fatigue;
  ros::Timer timer_overall_fatigue;
  ros::Timer timer_frustration;
  
  ros::Timer timer_robot_batt_simulation;
  ros::Timer timer_laptop_batt_simulation;

  ros::Timer timer_pain;
  /**
  * publish variablrs
  *
  */
  std_msgs::Int32 t_tired_level_status;
  std_msgs::Int32 t_curiosity_level;
  std_msgs::Int32 t_laptop_fatigue_level_status;
  std_msgs::Int32 t_robot_fatigue_level_status;
  std_msgs::Int32 t_day_cicle_fatigue;
  std_msgs::Int32 t_rest_level;
  std_msgs::Int32 t_task_fatigue_level;
  std_msgs::Int32 t_time_frustration_level;
  std_msgs::Int32 t_task_frustration_level;
  std_msgs::Int32 t_pain_level;
  std_msgs::Int32 t_bumper_total_hits;
  
  /**
  * Node Hadler
  *
  */
   ros::NodeHandle n;
   

};//End of class motivational_component

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "motivational_component");

  ros::NodeHandle n("~");
  
  //Create an object of class motivational_component that will take care of everything
  //motivational_component MotObject;

  //Create an object of class motivational_component that will take care of everything
  motivational_component MotObject(n);
  
  ros::spin();

  return 0;
}
