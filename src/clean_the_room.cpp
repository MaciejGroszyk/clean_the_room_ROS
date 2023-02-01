#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <vector>
#include <string.h>
#include <fstream>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; //akcja move_base
move_base_msgs::MoveBaseGoal goal; //wiadomosc do wysylania celu



std::vector<float> goal_points_list_x; //lista wspolrzednych x celu
std::vector<float> goal_points_list_y; //lista wspolrzednych y celu
int goal_num; //aktualny numer celu
int len_vec; //dlugosc listy celu
bool finish_before_time_limit= true;
ros::Publisher *cleaner_state_ptr;  

//funckja do edycji wiadomosci move_base_msgs/MoveBaseGoal.msg
void editGoalPoint(float x_input, float y_input)
{
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x_input;
    goal.target_pose.pose.position.y = y_input;
    goal.target_pose.pose.orientation.w = 1.0;
}

//funkcja uzupelniajaca globalne funkcje wspolrzednych x, y celu
int generateGoalList(float x_start, float x_end, float y_start, float y_end)
{
    float x_increment_by= 0.2; //wspolczynnik inkrementacji kolejnej wspolrzednej x
    float y_increment_by= 0.3; //wspolczynnik inkrementacji kolejnej wspolrzednej y
    // y_increment_by= y_end-y_start-0.1; //wersja 2 punkty na osi y
    int i=0;
    goal_points_list_x.clear();
    goal_points_list_y.clear();
    bool direction = true;
    for(float x=x_start; x <=x_end; x+= x_increment_by)
    {
      if (direction)
      {
        for(float y=y_start; y <=y_end; y+= y_increment_by)
        {
          goal_points_list_x.push_back(x);
          goal_points_list_y.push_back(y);
          i+=1;
        }
      }
      else 
      {
        for(float y=y_end; y>=y_start; y-= y_increment_by)
        {
          goal_points_list_x.push_back(x);
          goal_points_list_y.push_back(y);
          i+=1;
        }
      }
      direction= !direction;
    }
    return i; //zwraca dlugosc listy
}

//wysylanie nowego celu
void sendNewGoal()
{
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  editGoalPoint(goal_points_list_x[goal_num], goal_points_list_y[goal_num]);
  ROS_INFO("New goal %f, %f", goal_points_list_x[goal_num], goal_points_list_y[goal_num]);
  ROS_INFO("Gol number/All goals (%i/%i)", goal_num+1, len_vec);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  finish_before_time_limit=  ac.waitForResult(ros::Duration(20));

}

//subscriber na temacie "/move_base/status"
void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status)
{
  if (!status->status_list.empty()) 
  {
    //zczytanie wartosci status
    actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
    int s = goalStatus.status;
    ROS_INFO("%i ", s);
    std_msgs::String msg;
    std::stringstream ss;

    // wyslanie nowego celu, gdy cel został wycofany (2), osiągnięty (3), opuszczony (4), odrzucony (5)
    // oraz nie osiagnieto wszystkich celow
    if (((s==2 || s==3 || s==4 || s==5) && (goal_num< len_vec-1)) || (!finish_before_time_limit && goal_num!=0))
    {
      goal_num+=1;
      sendNewGoal();
      
      ss << "cleaning";
      msg.data = ss.str();
      cleaner_state_ptr -> publish(msg);
    }
    else if (goal_num ==  len_vec-1)
    {
      ss << "cleaning_ended";
      ROS_INFO("%i, %i ", goal_num, len_vec);
      msg.data = ss.str();
      cleaner_state_ptr -> publish(msg);

    }
  }
}

void roomNameCallback(const std_msgs::String::ConstPtr& room_name)
{
  std::string room = room_name -> data.c_str();

  if (room =="stolowka")
  {
    len_vec = generateGoalList(2.5, 7, 0.5, 4.5);
    sendNewGoal();
  }
  else if (room=="biuro")
  {
    len_vec = generateGoalList(5.2, 6.8, -5, -1);
    sendNewGoal();
  }
  else if (room=="smietnik")
  {
    len_vec = generateGoalList(0.3, 1.8, 1.5, 4.5);
    sendNewGoal();
  }
  else if (room=="kuchnia")
  {
    len_vec = generateGoalList(-4.5, -0.5, 0.5, 4.5);
    sendNewGoal();
  }
  else if (room=="szatnia")
  {
    len_vec = generateGoalList(-7, -5.5, 1.5, 4.5);
    sendNewGoal();

  }
  else if (room=="lazienka")
  {
    len_vec = generateGoalList(-7, -6, -3, 0.5);
    sendNewGoal();

  }
  else if ( room == "dock")
  {
    std::ifstream in("/home/mateusz/ima_ws/src/ima_22l_lab6_groszyk_zembron/package_groszyk_zembron/dock_param/data.txt"); // input

    float x;
    float y;

    in >> x >>  y;
    ROS_INFO("%f, %f", x, y);
    in.close();
    goal_points_list_x.clear();
    goal_points_list_y.clear();

    goal_points_list_x.push_back(x);
    goal_points_list_y.push_back(y);
    len_vec = 1;
    goal_num = 0;
    sendNewGoal();

  }
  else
  {
    ROS_INFO("False room name");
  }

  //wyslanie pierwszej wiadomosci
}
void obstacleCallback(const std_msgs::Float32MultiArray::ConstPtr& obstacle_pose)
{
    float x_obs = obstacle_pose ->data[0];
    float y_obs = obstacle_pose ->data[1];

    ROS_INFO("Obstacle !!! %f, %f", x_obs, y_obs );

    for (int i; i <goal_points_list_x.size(); ++i )
    {
        float x_subg = goal_points_list_x[i];
        float y_subg = goal_points_list_y[i];

        float obstacle_distance = sqrt(pow((x_subg - x_obs ),2)+pow((y_subg - y_obs ),2));
        if (obstacle_distance< 2)
        {
            int distance_sign = (x_subg - x_obs)/abs(x_subg - x_obs);
            goal_points_list_x[i] = 0.1*distance_sign*goal_points_list_x[i];
            ROS_INFO("New point !!! new_x:%f, prev_x: %f", goal_points_list_x[i], x_subg );
        }
        ROS_INFO("obstacle_distance: %f", obstacle_distance);
    }
}


int main(int argc, char** argv){

  //inicjalizacja
  ros::init(argc, argv, "RoomCleaning");

  //stworzenie subscribera
  ros::NodeHandle nH; 
  ros::Subscriber sub;
  ros::Subscriber goal_sub;
  ros::Subscriber obstacle_subscriber;
  ros::Publisher cleaner_state = nH.advertise<std_msgs::String>("RoomCleanerState", 1000);
  cleaner_state_ptr = &cleaner_state;
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  sub = nH.subscribe("/move_base/status", 1000, statusCallback);
  goal_sub = nH.subscribe("/room_name_publisher", 1000, roomNameCallback); 
  obstacle_subscriber = nH.subscribe("/obstacle_loc_publisher", 10, obstacleCallback);


  //zmiana tolerancji kąta rotacji
  system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS yaw_goal_tolerance 6.30");
  system("rosrun dynamic_reconfigure dynparam set /move_base/DWAPlannerROS xy_goal_tolerance 0.1");

  goal_num=0; //ustawienie numeru celu na początkowy
  len_vec = 0;

  //generacja listy wspolrzednych x,y zaleznie od parametru wejsciowego programu

  ros::spin();
  return 0;

}
