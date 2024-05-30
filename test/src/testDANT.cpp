#include <iostream>
#include <deque>
#include <string>
#include <vector>
using namespace std;

class point {
public:
    point( int a = 0, int b = 0 ) { x = a; y = b; }
    bool operator ==( const point& o ) { return o.x == x && o.y == y; }
    point operator +( const point& o ) { return point( o.x + x, o.y + y ); }
    int x, y;
};
std::deque<point> path;


void get_postition(string &s, int &cur_x, int &cur_y){ // x: 10, y: 20
  size_t commaPos = s.find(',');
  size_t colonPos = s.find(':');
   cur_x = std::stoi(s.substr(colonPos + 2, commaPos - colonPos - 2));
  colonPos = s.find(':', commaPos);
   cur_y = std::stoi(s.substr(colonPos + 2));


}
// // output: msg1 for x, msg2 for y
void Pub_msg_position(string &s, int cur_x, int cur_y,  deque<point> path){

  int sumx=0;
  int sumy=0;

  int sumx_cur=0;
  int sumy_cur=0;
  std::deque<point> tmp(path);

    int i = 0;
    point next;
    point cur ;
    vector<vector<int>> road;
    while(!tmp.empty() && i<path.size() -1){
         next = tmp[1];
         cur =  tmp[0];
        sumx_cur = sumx;
        if(abs( next.x - cur.x) >=1 ){
          sumx_cur = sumx;
          sumx+=(abs(next.x - cur.x));
          cout<< "Sum Y: " << sumy <<endl;
        }
        //cout<< "Sum X: " << sumx <<endl;
        // if(sumx_cur==sumx){
        //   cout<< "Sum X: " << sumx <<endl;

        // }
        sumy_cur = sumy;
        if(abs( next.y - cur.y) >=1){
          sumy_cur = sumy;
          sumy+=(abs((next.y - cur.y)));
          cout<< "Sum X: " << sumx <<endl;
        }
        if(sumy_cur == sumy){
          cout<< "Sum Y: " << sumy <<endl;

        }
        
        tmp.pop_front();
        i++;

        //cout<< "Sum Y: " <<sumy <<endl;
        //
    }

//  }
}void Pub_msg_position1(string &s, int cur_x, int cur_y, deque<point> path) {
    int sumx = 0;
    int sumy = 0;

    int sumx_cur = 0;
    int sumy_cur = 0;
    std::deque<point> tmp(path);

    int i = 0;
    int j = 0;
    point next;
    point cur;
    vector<pair<char, int>> offsets;
    next = tmp[j+1];
    cur = tmp[i];
    bool first = true;
    while (i < path.size() - 1) {

        int x_diff = abs(next.x - cur.x);
        int y_diff = abs(next.y - cur.y);

        if (x_diff >= 1) {
            // offsets.emplace_back('x', x_diff);
            // sumx += x_diff;
            next = tmp[j++];
            offsets.emplace_back('y', y_diff);
            continue;
        }
        else if(y_diff >= 1){
          offsets.emplace_back('x', x_diff);
          next = tmp[j++]; 
          continue;
        }
        
          cur = tmp[i];
          x_diff = abs(next.x - cur.x);
          offsets.emplace_back('x', x_diff);
        

       
          cur = tmp[i++];
          offsets.emplace_back('y', y_diff);
         //   next = tmp[i++];
            
        i++;
        first = false;
    }

    bool firstOffset = true;
    for (const auto& offset : offsets) {
        if (!firstOffset) {
            cout << ",";
        } else {
            firstOffset = false;
        }
        cout << "(" << offset.first << "," << offset.second << ")";
    }
    cout << endl;
}

void Pub_msg_position2(string &s, int cur_x, int cur_y, deque<point> path) {
    int sumx = 0;
    int sumy = 0;

    std::deque<point> tmp(path);

    int i = 0;
    point next;
    point cur;
    vector<pair<char, int>> offsets;

    while (!tmp.empty() && i < path.size() - 1) {
        next = tmp[1];
        cur = tmp[0];

        int x_diff = abs(next.x - cur.x);
        int y_diff = abs(next.y - cur.y);

        if (x_diff >= 1) {
            offsets.emplace_back('x', x_diff);
            sumx += x_diff;
        }

        if (y_diff >= 1) {
            offsets.emplace_back('y', y_diff);
            sumy += y_diff;
        }

        tmp.pop_front();
        i++;
    }

    bool firstOffset = true;
    for (const auto& offset : offsets) {
        if (!firstOffset) {
            cout << ",";
        } else {
            firstOffset = false;
        }
        cout << "(" << offset.first << "," << offset.second << ")";
    }
    cout << endl;
}

int main() {

    // Add some points to the path
    // path.push_back(point(1, 1));
    // path.push_back(point(2, 1));
    // path.push_back(point(2, 2));
    // path.push_back(point(2, 3));

    path.push_back(point(1, 1));
    path.push_back(point(1, 2));
    path.push_back(point(1, 3));
    path.push_back(point(2, 3));
    path.push_back(point(2, 4));
    int cur_x = 0;
    int cur_y = 0;
    // Print the path
    std::cout << "Path: ";
    for (const auto& p : path) {
        std::cout << "(" << p.x << ", " << p.y << ") ";
    }
    std::cout << std::endl;
    string msg_data = {"x: 1, y: 1"};
    get_postition(msg_data,cur_x,cur_y  );
    Pub_msg_position1(msg_data, cur_x,cur_y, path);
   // ::cout << "Value 1: " << cur_x << std::endl;
  //std::cout << "Value 2: " << cur_y << std::endl;
    return 0;
}
