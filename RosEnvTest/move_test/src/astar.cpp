#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"
#include <iostream>
#include <cmath>
#include <list>
#include <algorithm>
#include <deque>

const int OFFSET = -10;
struct quarternion
{
  double w;
  double x;
  double y;
  double z;
};


void TrajectoryPlanning(float DesirePosition, float TOTALTIME);

/////////////////////////////////////a-star////////////////////////////////////

class point {
public:
    point( int a = 0, int b = 0 ) { x = a; y = b; }
    bool operator ==( const point& o ) { return o.x == x && o.y == y; }
    point operator +( const point& o ) { return point( o.x + x, o.y + y ); }
    int x, y;
};

class map{
public:
    map() {
        char t[20][20];
        // {
        //     {0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0},
        //     {0, 0, 0, 0, 1, 1, 1, 0}, {0, 0, 1, 0, 0, 0, 1, 0},
        //     {0, 0, 1, 0, 0, 0, 1, 0}, {0, 0, 1, 1, 1, 1, 1, 0},
        //     {0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}
        // };
        w = h = 20;

        for( int r = 0; r < h; r++ )
            for( int s = 0; s < w; s++ )
                t[s][r] = 0;
//        t[0][2]=1; t[1][2]=1; t[2][2]=1; t[3][2]=1; t[4][2]=1; t[5][2]=1; t[15][0]=1;
//        t[15][1]=1; t[15][2]=1; t[15][3]=1; t[15][4]=1; t[15][5]=1; t[15][6]=1; t[15][7]=1;
//        t[15][8]=1; t[14][6]=1; t[13][6]=1; t[12][6]=1; t[11][6]=1; t[10][6]=1; t[9][6]=1;
//        t[8][6]=1; t[7][6]=1; t[6][6]=1;
        for( int r = 0; r < h; r++ )
            for( int s = 0; s < w; s++ )
                m[s][r] = t[r][s];
    }
    int operator() ( int x, int y ) { return m[x][y]; }
    char m[20][20];
    int w, h;
};


class node {
public:
    bool operator == (const node& o ) { return pos == o.pos; }
    bool operator == (const point& o ) { return pos == o; }
    bool operator < (const node& o ) { return dist + cost < o.dist + o.cost; }
    point pos, parent;
    int dist, cost;
};


class aStar {
public:
    aStar() {
        neighbours[0] = point( -1, -1 ); neighbours[1] = point(  1, -1 );
        neighbours[2] = point( -1,  1 ); neighbours[3] = point(  1,  1 );
        neighbours[4] = point(  0, -1 ); neighbours[5] = point( -1,  0 );
        neighbours[6] = point(  0,  1 ); neighbours[7] = point(  1,  0 );
    }

    int calcDist( point& p ){
        // need a better heuristic
        int x = end.x - p.x, y = end.y - p.y;
        return( abs(x) + abs(y) );
    }

    bool isValid( point& p ) {
        return ( p.x >-1 && p.y > -1 && p.x < m.w && p.y < m.h );
    }

    bool existPoint( point& p, int cost ) {
        std::list<node>::iterator i;
        i = std::find( closed.begin(), closed.end(), p );
        if( i != closed.end() ) {
            if( ( *i ).cost + ( *i ).dist < cost ) return true;
            else { closed.erase( i ); return false; }
        }
        i = std::find( open.begin(), open.end(), p );
        if( i != open.end() ) {
            if( ( *i ).cost + ( *i ).dist < cost ) return true;
            else { open.erase( i ); return false; }
        }
        return false;
    }

    bool fillOpen( node& n ) {
        int stepCost, nc, dist;
        point neighbour;

        for( int x = 0; x < 8; x++ ) {
            // one can make diagonals have different cost
            stepCost = x < 4 ? 7 : 5;
            neighbour = n.pos + neighbours[x];
            if( neighbour == end ) return true;

            if( isValid( neighbour ) && m( neighbour.x, neighbour.y ) != 1 ) {
              if( m( n.pos.x, neighbour.y ) != 1 && m( neighbour.x, n.pos.y ) != 1 ){
                nc = stepCost + n.cost;
                dist = calcDist( neighbour );
                if( !existPoint( neighbour, nc + dist ) ) {
                    node m;
                    m.cost = nc; m.dist = dist;
                    m.pos = neighbour;
                    m.parent = n.pos;
                    open.push_back( m );
                }
              }
            }
        }
        return false;
    }

    bool search( point& s, point& e, map& mp ) {
        node n; end = e; start = s; m = mp;
        n.cost = 0; n.pos = s; n.parent = 0; n.dist = calcDist( s );
        open.push_back( n );
        while( !open.empty() ) {
            open.sort();
            node n = open.front();
            open.pop_front();
            closed.push_back( n );
            if( fillOpen( n ) ) return true;
        }
        return false;
    }

    int path( std::deque<point>& path ) {
        path.push_front( end );
        int cost = 1 + closed.back().cost;
        path.push_front( closed.back().pos );
        point parent = closed.back().parent;

        for( std::list<node>::reverse_iterator i = closed.rbegin(); i != closed.rend(); i++ ) {
            if( ( *i ).pos == parent && !( ( *i ).pos == start ) ) {
                path.push_front( ( *i ).pos );
                parent = ( *i ).parent;
            }
        }
        path.push_front( start );
        return cost;
    }

    map m; point end, start;
    point neighbours[8];
    std::list<node> open;
    std::list<node> closed;
};
/////////////////////////////////////////////////////////////////////////////////////////

//int main( int argc, char* argv[] ) {
//    map m;
//    int offset = -10;
//    point s, e( 7, 7 );
//    aStar as;

//    if( as.search( s, e, m ) ) {
//        std::list<point> path;
//        int c = as.path( path );
//        for( int y = -1; y < 9; y++ ) {
//            for( int x = -1; x < 9; x++ ) {
//                if( x < 0 || y < 0 || x > 7 || y > 7 || m( x, y ) == 1 )
//                    std::cout << "1";
//                else {
//                    if( std::find( path.begin(), path.end(), point( x, y ) )!= path.end() )
//                        std::cout << "x";
//                    else std::cout << "0";
//                }
//            }
//            std::cout << "\n";
//        }

//        std::cout << "\nPath cost " << c << ": ";
//        for( std::list<point>::iterator i = path.begin(); i != path.end(); i++ ) {
//            std::cout<< "(" << ( *i ).x << ", " << ( *i ).y << ") ";
//        }
//    }
//    std::cout << "\n\n";
//    return 0;
//}

geometry_msgs::Twist robot_msg;
std_msgs::Float32 p_time;
std_msgs::Float32 r_time;

geometry_msgs::Twist move_func(double x, double y, double dest_x, double dest_y);
point s, e;
aStar as;

map mapp;
int flag_obs=1;
int flag_search=0;
int flag_dest=1;
int dx, dy, rx, ry;
int len;
int cnt = 1;
int flag_runtime = 0;
int flag_waiting = 0;
double r_s;
std::deque<point> path;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // return desired velocity
    pub_1 = n_.advertise<std_msgs::Float32>("process_time",10);
    pub_2 = n_.advertise<std_msgs::Float32>("run_time",10);

    sub_ = n_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",10,&SubscribeAndPublish::callback, this);
    ros::Subscriber sub_new = nh.subscribe("merged_coordinates", 10, &SubscribeAndPublish::processCallback, this);


    // cur_pos_sub_ = n_.subscribe<gazebo_msgs::ModelStates>("/cur_pos",10,&SubscribeAndPublish::callback_cur, this);

  }
void proceesCallback(const your_package_name::MergedCoordinates::ConstPtr& msg){
  if(flag_obs==1){ 
      r_s = ros::WallTime::now().toSec();
      as.closed.clear();
      as.open.clear();
      for(int i=0;i<20;i++){
        for(int j=0;j<20;j++){
          mapp.m[i][j] = 0;
        }
      }
      int i=1;
      len = i+2;
      ROS_INFO_STREAM("obstacles detected");
      flag_obs=0;
      flag_search=1;
    }
    if(flag_obs==0 && flag_search==1){
      path.clear();
      double p_s = ros::WallTime::now().toSec();
      if(as.search(s,e,mapp)){
        int c = as.path( path );

        double p_e = ros::WallTime::now().toSec();
        p_time.data = p_e - p_s;
        pub_1.publish(p_time);

        ROS_INFO_STREAM("success");
//          if(cnt != 1){
//            path.pop_front();
//          }
//          cnt += 1;
        path.pop_front();
        flag_search=0;
        flag_waiting=1;
      }
      else{
        ROS_INFO_STREAM("Error");
      }
    }
    else if(flag_obs==0 && flag_search==0 && flag_waiting==1){ // RUN
      ROS_INFO_STREAM("Robot name:" << msg->name.back());
      ROS_INFO_STREAM("Position:" << msg->x << "," << msg->x << "," );

      double dest_x = path.begin()->x+OFFSET;   /// TODO:
      double dest_y = path.begin()->y+OFFSET;
      double x = msg->x                       /// TODO: to get cur pos x
      double y = msg->y                       /// TODO: to get cur pos y
      double dist = sqrt(pow((dest_x-x),2)+pow((dest_y-y),2));

      //int c = as.path( path );
      if(path.empty()){
        robot_msg.linear.x=0;
        robot_msg.angular.z=0;
        double r_e = ros::WallTime::now().toSec();
        r_time.data = r_e - r_s;
        pub_2.publish(r_time);
        pub_.publish(robot_msg);
        flag_waiting = 0;
        //ros::Duration(0.5).sleep();
      }
      else{
        robot_msg = move_func(x, y, dest_x, dest_y);

        pub_.publish(robot_msg);

      }
    }
    if (flag_obs==0 && flag_search==0 && flag_waiting==0){ // robot waitting to new p
      ROS_INFO_STREAM("Waiting...");
    }
}


private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Publisher pub_1;
  ros::Publisher pub_2;
  ros::Subscriber sub_;
};

//void callback(const gazebo_msgs::ModelStates::ConstPtr& msg);

//ros::NodeHandle nh;
//ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
//ros::Subscriber chatter_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",5,callback);


//TODO
void TrajectoryPlanning(float DesirePosition, float TOTALTIME){ //path planning generator
  /***********************************************************
					Trapezoidal Trajectory Planning
          _________v=const___________
         /                           \
        /                             \   
  accel segment                   decel segment
  ************************************************************/

  // return desired speed 
}
// void get_postition(string &s, int &x, int &y){ // x: 10, y: 20 // GET DATA FROM UART

//   size_t commaPos = s.find(',');
//   size_t coloPos = s.find(':');
//   int m_x = std::stoi(input.substr(colonPos + 2, commaPos - colonPos - 2));
//   colonPos = input.find(':', commaPos);
//   int value2 = std::stoi(input.substr(colonPos + 2));

//   std::cout << "Value 1: " << value1 << std::endl;
//   std::cout << "Value 2: " << value2 << std::endl;
// }
// output: msg1 for x, msg2 for y
geometry_msgs::position Pub_msg_position(){
  std_msgs::String msg;  
  int cur_x=0;
  int cur_y=0;
  int sumx=0;
  int sumy=0;
  std::deque<point> tmp(path);
  
  msg.data = my_serial.read(my_serial.available()); // READING UART/ Format: "x: , y: "
  get_postition(msg.data,&cur_x,&cur_y);

  double dest_x = tmp.begin()->x+OFFSET;
  double dest_y = tmp.begin()->y+OFFSET;
  while(!tmp.empty()){
    if()
    for (int num : tmp) {
        std::cout << num.x << " ";
        std::cout << num.y << " ";
        if(num.x )

    }
    for (size_t i = 0; i < tmp.size() -1; i++) {
        int next = tmp[i + 1];
        int curr = tmp[0];
        while(abs(cur.x - next.x) >1){
          cout<<"X is changing";
          sumx+=(abs(cur.x - next.x));
        }

        while(abs(cur.y - next.y) >1){
          cout<<"Y is changing";
          sumy+=(abs(cur.y - next.y));
        }
        if()
    }
  }
}

geometry_msgs::Twist move_func(double x, double y,  double dest_x, double dest_y){

 
  double vel_x;
  double vel_y;
  double t;
  bool lift = 0;  // if lift =0 : no lift-> robot go  along w the x axis, lift =1 -> robot go  along w the y axis

  t = 1;

  
  std_msgs::String msg;  
  msg.data = my_serial.read(my_serial.available()); // READING UART
  msg.x;
  msg.y;
  double dist = sqrt(pow((dest_x-msg.x),2)+pow((dest_y-msg.y),2));
  double dist_x = 0;
  double dist_y = 0;
  if (dist < 0.1) {
    robot_msg.linear.x = 0;
    robot_msg.linear.y = 0;
    robot_msg.angular.z = 0;
    path.pop_front();
  }
  else{
    if (std::abs(dest_x-x) > 1 ){
      // Di chuyển theo trục x
      robot_msg.linear.x = 0.8;
      robot_msg.linear.y = 0;
      robot_msg.angular.z = 0;
    }
    else{
      // Di chuyển theo trục y
      robot_msg.linear.x = 0;
      robot_msg.linear.y = 0.8;
      robot_msg.angular.z = 0;
    }
  }

  return robot_msg;
}


// void Pub_msg_position1(string &s, int cur_x, int cur_y, deque<point> path) {
//     int sumx = 0;
//     int sumy = 0;

//     int sumx_cur = 0;
//     int sumy_cur = 0;
//     std::deque<point> tmp(path);

//     int i = 0;
//     int j = 0;
//     point next;
//     point cur;
//     vector<pair<char, int>> offsets;
//     next = tmp[j+1];
//     cur = tmp[i];
//     bool first = true;
//     while (i < path.size() - 1) {

//         int x_diff = abs(next.x - cur.x);
//         int y_diff = abs(next.y - cur.y);

//         if (x_diff >= 1) {
//             // offsets.emplace_back('x', x_diff);
//             // sumx += x_diff;
//             next = tmp[j++];
//             offsets.emplace_back('y', y_diff);
//             continue;
//         }
//         else if(y_diff >= 1){
//           offsets.emplace_back('x', x_diff);
//           next = tmp[j++]; 
//           continue;
//         }
        
//           cur = tmp[i];
//           x_diff = abs(next.x - cur.x);
//           offsets.emplace_back('x', x_diff);
        

       
//           cur = tmp[i++];
//           offsets.emplace_back('y', y_diff);
//          //   next = tmp[i++];
            
//         i++;
//         first = false;
//     }

//     bool firstOffset = true;
//     for (const auto& offset : offsets) {
//         if (!firstOffset) {
//             cout << ",";
//         } else {
//             firstOffset = false;
//         }
//         cout << "(" << offset.first << "," << offset.second << ")";
//     }
//     cout << endl;
// }



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pub");

  ros::Time::init();
  ros::Rate loop_rate(10);
  double robot_posx = 0;
  double robot_posy = 0;
  SubscribeAndPublish SAPObject;


    while (ros::ok())
    {
  //    msg.linear.x = 0;
  //    msg.angular.z = 0;
  //    chatter_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
    }


  return 0;

}

