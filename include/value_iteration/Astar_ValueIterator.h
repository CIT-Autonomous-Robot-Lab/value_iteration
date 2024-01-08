#ifndef ASTAR_VALUE_ITERATOR_
#define ASTAR_VALUE_ITERATOR_

#include "ValueIterator.h"
#include <vector>
#include <geometry_msgs/Pose.h>

namespace value_iteration{

class Node {
public:
    int x; // ノードの x 座標
    int y; // ノードの y 座標
    double g; // スタートからの実際のコスト
    double h; // ヒューリスティック推定コスト
    double f; // 総コスト
    Node* parent; // 親ノードへのポインタ

    Node() : x(0), y(0), g(0.0), h(0.0), f(0.0), parent(nullptr) {}

    Node(int x_, int y_) : x(x_), y(y_), g(0.0), h(0.0), f(0.0), parent(nullptr) {}

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }

};

class Astar_ValueIterator : public ValueIterator{
public:
    ValueIterator* vi;
	Astar_ValueIterator(vector<Action> &actions, int thread_num);
 
	std::vector<geometry_msgs::Pose> calculateAStarPath(nav_msgs::OccupancyGrid &map, geometry_msgs::Pose start, geometry_msgs::Pose goal);

    void setMapWithOccupancyGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
	    double safety_radius, double safety_radius_penalty,
	    double goal_margin_radius, int goal_margin_theta);

    double calculateHeuristic(const Node& node, const Node& goal);

    Node getMinimumFValueNode(const std::vector<Node>& open_list);
    bool isGoalNode(const Node& node, const Node& goal);
    bool inClosedList(const Node& node, const std::vector<Node>& closed);
    void updateNodeCosts(Node& node, const Node& current, const Node& goal);
    void addOrUpdateOpenList(std::vector<Node>& open, Node& node);
    //std::vector<geometry_msgs::Pose> reconstructPath(const Node& goal);
    std::vector<geometry_msgs::Pose> calcFinalPath(const Node& goal, const std::vector<Node>& closed);
    std::vector<Node> getAdjacentNodes(const Node& current, const nav_msgs::OccupancyGrid& map);
    double getMoveCost(const Node& from, const Node& to);
    bool isInsideMap(int ix, int iy);
    bool isObstacle(int ix, int iy, const nav_msgs::OccupancyGrid& map);

    //void makeAstarValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold, 
	//		double x, double y, double yaw_rad);

    void cellDelta(double x, double y, double t, int &ix, int &iy, int &it);

    //void setGoal(double goal_x, double goal_y);

    void setWindowsOnAstarPath();

    void setAstarWindow(const Node& center_node);

    void makeAstarValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold,
		double x, double y, double yaw_rad);
    
    bool inAstarArea(int x, int y);

    //vector<int> convertAstarPathToStateIndex(const vector<Node>& nodePath);

    uint64_t actionCostAstar(State &s, Action &a);

    //void valueIterationAstarPath(const vector<int>& stateIndexPath);
    uint64_t valueIterationAstarPath(State &s);

    void astarValueIterationLoop(void);

    void valueIterationAstarPathWorker(int id);

    //void valueIterationAstarPathWorker(const vector<Node>& nodePath);

    //void setStateTransitionWorkerSub(Action &a, int it);

    //void setStateTransitionWorker(int it);

    //void setStateTransition(void);

    //void noNoiseStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);

    //void setStateValues(void);

    //void setState(const nav_msgs::OccupancyGrid &map, double safety_radius, double safety_radius_penalty);

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
    //std::vector<Node> astarPath;       
    std::vector<geometry_msgs::Pose> astarPath; // 経路を保持する変数
	int astar_ix_min_, astar_ix_max_, astar_iy_min_, astar_iy_max_;
	int astar_ixy_range_;
	double astar_xy_range_;
  //std::vector<Node> nodePath;
  //std::vector<State> states_;
  //uint64_t actionCostAstar(State &s, Action &a);
  //void Astar_valueIterationWorker(int times, int id);
    
};

}

#endif