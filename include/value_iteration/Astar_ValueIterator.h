#ifndef ASTAR_VALUE_ITERATOR_
#define ASTAR_VALUE_ITERATOR_

#include "ValueIterator.h"
#include <vector>
#include <geometry_msgs/Pose.h>

namespace value_iteration{

struct ContinuousNode {
  double x;
  double y;   
  double g;  
  double h;
  double f; 
  ContinuousNode* parent;

  ContinuousNode() : x(0), y(0), g(0.0), h(0.0), f(0.0), parent(nullptr) {}

  ContinuousNode(double _x, double _y) 
    : x(_x), y(_y), g(0.0), h(0.0), f(0.0), parent(NULL) 
  {}

  bool operator==(const ContinuousNode& other) const {
    return x == other.x && y == other.y;
  }

  bool operator<(const ContinuousNode& other) const {
    if (x != other.x) {
      return x < other.x; 
    }
    return y < other.y;
  }

};


class Astar_ValueIterator : public ValueIterator{
public:
    ValueIterator* vi;
	Astar_ValueIterator(std::vector<Action> &actions, int thread_num);
 
	std::vector<ContinuousNode> calculateAStarPath(geometry_msgs::Pose start, ContinuousNode goal);

    void setMapWithOccupancyGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
	    double safety_radius, double safety_radius_penalty,
	    double goal_margin_radius, int goal_margin_theta);

    double calculateHeuristic(const ContinuousNode& node, const ContinuousNode& goal);

    ContinuousNode getMinimumFValueNode(const std::set<ContinuousNode>& open_list);

    bool isGoalNode(const ContinuousNode& node, const ContinuousNode& goal);

    bool inClosedList(const ContinuousNode& node, const std::set<ContinuousNode>& closed);

    void updateNodeCosts(ContinuousNode& node, const ContinuousNode& current, const ContinuousNode& goal);

    void addOrUpdateOpenList(std::set<ContinuousNode>& open, ContinuousNode& node);

    //std::vector<geometry_msgs::Pose> reconstructPath(const Node& goal);

    //std::vector<geometry_msgs::Pose> calcFinalPath(const ContinuousNode& goal, const std::vector<ContinuousNode>& closed);

    std::set<ContinuousNode> getAdjacentNodes(const ContinuousNode& current);

    double getMoveCost(const ContinuousNode& from, double to_x, double to_y);

    //bool isInsideMap(int ix, int iy);

    //bool isObstacle(int ix, int iy, const nav_msgs::OccupancyGrid& map);

    //bool isColliding(double x, double y, const nav_msgs::OccupancyGrid& map);

    //void cellDelta(double x, double y, double t, int &ix, int &iy, int &it);

    void setGoal(double goal_x, double goal_y);

    //vector<int> convertAstarPathToStateIndex(const vector<Node>& nodePath);

    //uint64_t actionCostAstar(State &s, Action &a);

    //void valueIterationAstarPath(const vector<int>& stateIndexPath);
    //uint64_t valueIterationAstarPath(State &s);

    //void AstarvalueIterationLoop(void);

    //void valueIterationAstarPathWorker(const vector<Node>& nodePath);

    //void setStateTransitionWorkerSub(Action &a, int it);

    //void setStateTransitionWorker(int it);

    //void setStateTransition(void);

    //void noNoiseStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);

    //void setStateValues(void);

    //void setState(const nav_msgs::OccupancyGrid &map, double safety_radius, double safety_radius_penalty);

    //int getIndex(int x, int y);

    //void setSweepOrders(void);

    //bool finished(std_msgs::UInt32MultiArray &sweep_times, std_msgs::Float32MultiArray &deltas);



    //void setPathStates(const nav_msgs::OccupancyGrid &map, double safety_radius, double safety_radius_penalty);

    //uint64_t Astar_valueIteration(State &s);
    //uint64_t actionCost(State &s, Action &a);
    //void Astar_valueIterationWorker(int id);
    //void astarValueIterationLoop(void);
    //State convertPoseToState(const geometry_msgs::Pose &pose);

private:
  double goal_x_; 
  double goal_y_;  
  //std::set<ContinuousNode> open_list;
  //std::set<ContinuousNode> closed_list;     
  std::vector<geometry_msgs::Pose> astarPath; // 経路を保持する変数
  //std::vector<Node> nodePath;
  //std::vector<State> states_;
  //uint64_t actionCostAstar(State &s, Action &a);
  //void Astar_valueIterationWorker(int times, int id);
    
};

}

#endif