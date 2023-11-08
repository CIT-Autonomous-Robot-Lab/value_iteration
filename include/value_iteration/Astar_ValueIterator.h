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

    Node(int x_, int y_) : x(x_), y(y_), g(0.0), h(0.0), f(0.0), parent(nullptr) {}

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }

    State toState() const {
        // Node オブジェクトから State オブジェクトへの変換を行います。
        State state(x, y, 0, 0);  // 適切なパラメータを使って State オブジェクトを初期化してください
        state.optimal_action_ = nullptr;  // 任意の初期化が必要な場合は行ってください
        return state;
    }
};

class Astar_ValueIterator : public ValueIterator{
public:
	Astar_ValueIterator(vector<Action> &actions, int thread_num);
 
	std::vector<geometry_msgs::Pose> calculateAStarPath(nav_msgs::OccupancyGrid &map, geometry_msgs::Pose start, geometry_msgs::Pose goal);

    void setMapWithOccupancyGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
	    double safety_radius, double safety_radius_penalty,
	    double goal_margin_radius, int goal_margin_theta);

    double calculateHeuristic(const Node& node, const geometry_msgs::Pose& goal);

    Node getLowestFNode(const std::vector<Node>& nodes);
    bool reachedGoal(const Node& node, const geometry_msgs::Pose& goal);
    std::vector<geometry_msgs::Pose> buildPath(const Node& goalNode);
    std::vector<Node> getNeighbors(const Node& node, nav_msgs::OccupancyGrid &map); 
    bool containsNode(const std::vector<Node>& nodes, const Node& node);
    double calculateDistance(const Node& from, const Node& to);

    // ノードのオープンリストとクローズドリストを初期化
    std::vector<Node> openList;
    std::vector<Node> closedList;
    //std::vector<geometry_msgs::Pose> path;

    void performValueIterationForAStarPath(State& state, nav_msgs::OccupancyGrid& map);

    void astarValueIterationWorker(int id);

    Action *posToAction(double x, double y, double t_rad);
    void astarValueIterationLoop(nav_msgs::OccupancyGrid &map);

    nav_msgs::OccupancyGrid  map_;

    double getMapCost(const State& state, const nav_msgs::OccupancyGrid& map);
    double calculateMoveCost(const State& state, const Action& action, const nav_msgs::OccupancyGrid& map);
    double calculateRotateCost(const Action& action);
    int64_t calculateTotalCost(double move_cost, double rotate_cost);

    void makeAstarValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold, 
			double x, double y, double yaw_rad);
    //bool reached_goal(const Node& current, const geometry_msgs::Pose& goal);

private: 
    //void astarValueIterationLoop(void);
    int astar_ix_min_, astar_ix_max_, astar_iy_min_, astar_iy_max_;
    int64_t calculateActionCostForAStarState(const State& state, const Action& action, const nav_msgs::OccupancyGrid &map);
    
};

}

#endif