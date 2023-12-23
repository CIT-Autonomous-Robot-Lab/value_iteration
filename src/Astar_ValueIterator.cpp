#include "value_iteration/Astar_ValueIterator.h"
#include <thread>

namespace value_iteration{

Astar_ValueIterator::Astar_ValueIterator(std::vector<Action> &actions, int thread_num) : ValueIterator(actions, thread_num) 
{
  //states_.resize(cell_num_x_ * cell_num_y_ * cell_num_t_, State(0, 0, 0, nav_msgs::OccupancyGrid(), 0, 0, 0));
  ROS_INFO("SET ASTAR");
}

void Astar_ValueIterator::setMapWithOccupancyGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta)
{
	ValueIterator::setMapWithOccupancyGrid(map, theta_cell_num, safety_radius, safety_radius_penalty,
			goal_margin_radius, goal_margin_theta);
  
  cell_num_t_ = theta_cell_num;
	goal_margin_radius_ = goal_margin_radius;
	goal_margin_theta_ = goal_margin_theta;

	cell_num_x_ = map.info.width;
	cell_num_y_ = map.info.height;

	xy_resolution_ = map.info.resolution;
	t_resolution_ = 360/cell_num_t_;

	map_origin_x_ = map.info.origin.position.x;
	map_origin_y_ = map.info.origin.position.y;
	map_origin_quat_ = map.info.origin.orientation;

  //setState(map, safety_radius, safety_radius_penalty);
	//setStateTransition();
  //setSweepOrders();

  //states_.clear();
	//int margin = (int)ceil(safety_radius/xy_resolution_);
  //setState(map, safety_radius, safety_radius_penalty);
    
}

double Astar_ValueIterator::calculateHeuristic(const ContinuousNode& node, const ContinuousNode& goal) {

  // ユークリッド距離を使用
  double dx = node.x - goal.x; 
  double dy = node.y - goal.y;
  //ROS_INFO("Goal: (%f, %f)", goal.x, goal.y);

  return sqrt(dx*dx + dy*dy); 

}

std::vector<ContinuousNode> Astar_ValueIterator::calculateAStarPath(geometry_msgs::Pose start, ContinuousNode goal)
{
  std::vector<ContinuousNode> path;
  // 開始ノード
  double start_x = -3.0;
  double start_y = 1.0;
  ContinuousNode start_node(start_x, start_y);
  ROS_INFO("Start node: (%f, %f)", start_node.x, start_node.y);

  // ゴールノード
  //goal_x_ = goal.position.x;
  //goal_y_ = goal.position.y;
  //ROS_INFO("Goal: (%f, %f)", goal_x_, goal_y_);

  //int goal_x_int = (int)goal_x_; 
  //int goal_y_int = (int)goal_y_;
  double goal_x = goal.x;
  double goal_y = goal.y;
  ContinuousNode goal_node(goal_x, goal_y);

  //Node goal_node;
  //goal_node.x = goal_x_;
  //goal_node.y = goal_y_;
  ROS_INFO("Goal node: (%f, %f)",  goal_node.x,  goal_node.y);

  // openリスト
  std::set<ContinuousNode> open_list;

  // closedリスト 
  std::set<ContinuousNode> closed_list;

  // openリストにスタートノードを追加
  open_list.insert(start_node);

  // ゴールに到達するまで探索を繰り返す
  while(!open_list.empty()) {

    ContinuousNode current = getMinimumFValueNode(open_list);

    ROS_INFO("Current node: (%f, %f, %f)", current.x, current.y, current.f);
    
    if(isGoalNode(current, goal)) {
      // ゴールに到達したら終了
      break; 
    }

    open_list.erase(current);
    closed_list.insert(current);
    
    std::set<ContinuousNode> neighbors = getAdjacentNodes(current);

    //ROS_INFO("Found %lu adjacent nodes", neighbors.size());

    for(ContinuousNode node : neighbors) {

      if(inClosedList(node, closed_list)) 
        continue;
      
      updateNodeCosts(node, current, goal);
      
      addOrUpdateOpenList(open_list, node);   

    }

  }



  return path;

}

// goalをセット
void Astar_ValueIterator::setGoal(double goal_x, double goal_y)
{
	goal_x_ = goal_x;
	goal_y_ = goal_y;

	//ROS_INFO("GOAL: %f, %f", goal_x_, goal_y_);
  // ここで初期コスト設定
  //setStateValues();

  // ステータスやスレッド情報のクリア
  //thread_status_.clear();
  //setStateValues();
  //status_ = "calculating";

}

// f値が最小のノードを取得
ContinuousNode Astar_ValueIterator::getMinimumFValueNode(const std::set<ContinuousNode>& open_list)
{
    double min_f = std::numeric_limits<double>::max();
    ContinuousNode min_f_node;

    for (const ContinuousNode& node : open_list) {
        if (node.f < min_f) {
            min_f = node.f;
            min_f_node = node;
        }
    }

    return min_f_node;
}


// ノードがゴールノードか判定
bool Astar_ValueIterator::isGoalNode(const ContinuousNode& node, const ContinuousNode& goal) 
{
  //ROS_INFO("Goal node: (%d, %d)", goal.x, goal.y);
  return node.x == goal.x && node.y == goal.y; 
}

// ノードがclosedリストに含まれるか判定 
bool Astar_ValueIterator::inClosedList(const ContinuousNode& node, const std::set<ContinuousNode>& closed) 
{  
  return std::find(closed.begin(), closed.end(), node) != closed.end();
}
// f, g, h値の更新
void Astar_ValueIterator::updateNodeCosts(ContinuousNode& node, const ContinuousNode& current, const ContinuousNode& goal) 
{
  // g値更新
  node.g = current.g + getMoveCost(current, node.x, node.y);
  //node.g = current.g + 1;
  
  // h値更新
  node.h = calculateHeuristic(node, goal);  

  // f値更新
  node.f = node.g + node.h;

}

// 2つのノード間の移動コストを取得
double Astar_ValueIterator::getMoveCost(const ContinuousNode& from, double to_x, double to_y) 
{
  // ユークリッド距離等でコスト計算
  return sqrt(pow(to_x - from.x, 2) + pow(to_y - from.y, 2));  
}
// openリストへのノードの追加または更新
void Astar_ValueIterator::addOrUpdateOpenList(std::set<ContinuousNode>& open, ContinuousNode& node) 
{
  auto it = std::find(open.begin(), open.end(), node);
  if (it == open.end()) {
    // 見つからない場合はリストに追加
    open.insert(node);
  }
  else {
    // 見つかった場合は内容を更新
    //*it = node;
    open.erase(*it); 
    open.insert(node);
  }

}

// 隣接ノードを取得
std::set<ContinuousNode> Astar_ValueIterator::getAdjacentNodes(const ContinuousNode& current)
{
  std::set<ContinuousNode> neighbors;

  double search_resolution = 0.1; 

  for(double dx = -search_resolution; dx <= search_resolution; dx += search_resolution) {
    for (double dy = -search_resolution; dy <= search_resolution; dy += search_resolution) {
    
      double nx = current.x + dx;  
      double ny = current.y + dy;

      // 移動コスト計算
      double movement_cost = getMoveCost(current, nx, ny);

      ContinuousNode neighbor(nx, ny);
      neighbor.g = current.g + movement_cost;
      neighbors.insert(neighbor);

    }
  }

  return neighbors;

}

// 2つのノード間の移動コストを取得

// ゴールからスタートへの経路復元


// A*で計算されたパスに対して局所的な価値反復を実行



// A*パス(ノードリスト)を状態空間インデックスのベクトルに変換




} // namespace value_iteration