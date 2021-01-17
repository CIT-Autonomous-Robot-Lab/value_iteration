#ifndef VALUE_ITERATOR_
#define VALUE_ITERATOR_

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <fstream>
using namespace std;

class State{
public: 
	double _value;
	int _ix, _iy, _it;
	bool _free;
	bool _final_state;

	State(int x, int y, int theta, int map_value);
};

class Action{
public:
	string _name;
	double _delta_fw;  //forward traveling distance[m]
	double _delta_rot;  //rotation[deg]

	double _delta_fw_stdev;
	double _delta_rot_stdev;

	Action(string name, double fw, double rot);
};

class ValueIterator{
private: 
	vector<State> _states;
	vector<Action> _actions;

	double _cell_x_width, _cell_y_width, _cell_t_width;
	int _cell_x_num, _cell_y_num, _cell_t_num;

	int _center_state_ix, _center_state_iy;

	double _final_state_x, _final_state_y, _final_state_width;
public: 
	ValueIterator(nav_msgs::OccupancyGrid &map);

	void setAction(void);
	void outputPbmMap(void);
	void setFinalState(void);
	void outputValuePgmMap(void);
};

#endif
