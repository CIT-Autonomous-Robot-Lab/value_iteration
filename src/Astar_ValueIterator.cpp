#include "value_iteration/Astar_ValueIterator.h"

namespace value_iteration{

Astar_ValueIterator::Astar_ValueIterator(std::vector<Action> &actions, int thread_num) : ValueIterator(actions, thread_num)
{
    //ROS_INFO("SET ASTAR");
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
    
}

void Astar_ValueIterator::makeAstarValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold, 
		double x, double y, double yaw_rad)
{
	map.header.stamp = ros::Time::now();
	map.header.frame_id = "map";
	map.info.resolution = xy_resolution_;
	map.info.width = cell_num_x_;
	map.info.height = cell_num_y_;
	map.info.origin.position.x = map_origin_x_;
	map.info.origin.position.y = map_origin_y_;
	map.info.origin.orientation = map_origin_quat_;

  //ROS_INFO("Added map at %d", map.info.width);

  int it = (int)floor( ( ((int)(yaw_rad/M_PI*180) + 360*100)%360 )/t_resolution_ );

	for(int y=0; y<cell_num_y_; y++)
		for(int x=0; x<cell_num_x_; x++){
			int index = toIndex(x, y, it);
			double cost = (double)states_[index].total_cost_/prob_base_;
			if(cost < (double)threshold)
				map.data.push_back((int)(cost/threshold*250));
			else if(states_[index].free_)
				map.data.push_back(250);
			else 
				map.data.push_back(255);
		}

}

double Astar_ValueIterator::calculateHeuristic(const Node& node, const Node& goal) {

  // ユークリッド距離を使用
  double dx = node.x - goal.x; 
  double dy = node.y - goal.y;
  //ROS_INFO("Goal: (%f, %f)", goal.x, goal.y);

  return sqrt(dx*dx + dy*dy); 

}

std::vector<geometry_msgs::Pose> Astar_ValueIterator::calculateAStarPath(nav_msgs::OccupancyGrid &map, geometry_msgs::Pose start, geometry_msgs::Pose goal)
{
  // 開始ノード
  int start_x = -3.0;
  int start_y = 1.0;
  Node start_node(start_x, start_y);
  ROS_INFO("Start node: (%f, %f)", start_node.x, start_node.y);

  // ゴールノード
  goal_x_ = goal.position.x;
  goal_y_ = goal.position.y;
  ROS_INFO("Goal: (%f, %f)", goal_x_, goal_y_);

  int goal_x_int = (int)goal_x_; 
  int goal_y_int = (int)goal_y_;
  Node goal_node(goal_x_int, goal_y_int);

  //Node goal_node;
  //goal_node.x = goal_x_;
  //goal_node.y = goal_y_;
  ROS_INFO("Goal node: (%f, %f)",  goal_node.x,  goal_node.y);

  // openリスト
  std::vector<Node> open_list;

  // closedリスト 
  std::vector<Node> closed_list;

  // openリストにスタートノードを追加
  open_list.push_back(start_node);

  // ゴールに到達するまで探索を繰り返す
  while (!open_list.empty()) 
  {

    // openリストから評価値が最小のノードを取得
    Node current_node = getMinimumFValueNode(open_list);

    ROS_INFO("Current node: (%f, %f, %f)", current_node.x, current_node.y, current_node.f);

    // ゴールチェック
    if (isGoalNode(current_node, goal_node)) 
    {
      break;
    }

    // openリストから取り出したノードをclosedリストに追加
    closed_list.push_back(current_node);

    // openリストから取り出したノードを削除
    open_list.erase(
      std::remove(open_list.begin(), open_list.end(), current_node),
      open_list.end());

    // 隣接ノードを取得
    std::vector<Node> adjacent_nodes = getAdjacentNodes(current_node, map);

    //ROS_INFO("Found %lu adjacent nodes", adjacent_nodes.size());
        
    for (Node& node : adjacent_nodes) 
    {
      // closedリストにあるノードはスキップ
      if (inClosedList(node, closed_list)) 
      {
        continue;
      }

      // f, g, h値を更新
      updateNodeCosts(node, current_node, goal_node);
      
      // openリストに追加または更新
      addOrUpdateOpenList(open_list, node);

    }

  }

  // ゴールからスタートへの経路を復元
  auto path = calcFinalPath(goal_node, closed_list);

  ROS_INFO("Path found with %lu nodes", path.size());

  return path;

}

// goalをセット
void Astar_ValueIterator::setGoal(double goal_x, double goal_y)
{
	goal_x_ = goal_x;
	goal_y_ = goal_y;

	//ROS_INFO("GOAL: %f, %f", goal_x_, goal_y_);

}

// f値が最小のノードを取得
Node Astar_ValueIterator::getMinimumFValueNode(const std::vector<Node>& open_list)
{
    double min_f = std::numeric_limits<double>::max();
    Node min_f_node;

    for (const Node& node : open_list) {
        if (node.f < min_f) {
            min_f = node.f;
            min_f_node = node;
        }
    }

    return min_f_node;
}

// ノードがゴールノードか判定
bool Astar_ValueIterator::isGoalNode(const Node& node, const Node& goal) 
{
  //ROS_INFO("Goal node: (%d, %d)", goal.x, goal.y);
  return node.x == goal.x && node.y == goal.y; 
}

// ノードがclosedリストに含まれるか判定 
bool Astar_ValueIterator::inClosedList(const Node& node, const std::vector<Node>& closed) 
{  
  return std::find(closed.begin(), closed.end(), node) != closed.end();
}
// f, g, h値の更新
void Astar_ValueIterator::updateNodeCosts(Node& node, const Node& current, const Node& goal) 
{
  // g値更新
  node.g = current.g + getMoveCost(current, node);
  //node.g = current.g + 1;
  
  // h値更新
  node.h = calculateHeuristic(node, goal);  

  // f値更新
  node.f = node.g + node.h;

}
// openリストへのノードの追加または更新
void Astar_ValueIterator::addOrUpdateOpenList(std::vector<Node>& open, Node& node) 
{
  auto it = std::find(open.begin(), open.end(), node);
  if (it == open.end()) {
    // 見つからない場合はリストに追加
    open.push_back(node);
  }
  else {
    // 見つかった場合は内容を更新
    *it = node;
  }

}
// ゴールからスタートへの経路復元
std::vector<geometry_msgs::Pose> Astar_ValueIterator::calcFinalPath(const Node& goal, const std::vector<Node>& closed) {
  std::vector<geometry_msgs::Pose> path;
  Node current = goal;
  while(current.parent != nullptr) {
    geometry_msgs::Pose pose;
    pose.position.x = current.x;
    pose.position.y = current.y;
    path.insert(path.begin(), pose);
    current = *(current.parent);
  }
  return path;
}

bool Astar_ValueIterator::isObstacle(int x, int y, const nav_msgs::OccupancyGrid& map) {
  // mapデータから障害物か判定 
  return map.data[y * map.info.width + x] == 250; 
}

// map内に含まれるか判定
bool Astar_ValueIterator::isInsideMap(int ix, int iy) 
{
  //ROS_INFO("Added map at %d", cell_num_x_);
  //return ix < cell_num_x_ and iy < cell_num_y_ and ix != 0 and iy !=0;
  return ix < cell_num_x_ and iy < cell_num_y_;
}

// 隣接ノードを取得
std::vector<Node> Astar_ValueIterator::getAdjacentNodes(const Node& current, const nav_msgs::OccupancyGrid& map)
{
    std::vector<Node> adjacent_nodes;

    // 8方向の隣接座標
    int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    for(int i = 0; i < 8; i++) {
    
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        //ROS_INFO("Added neighbor at (%d, %d)", nx, ny);

        // 座標が範囲内かつ障害物ではない場合、ノードを追加
        // Stateクラスを使った障害物判定
        //int margin = 1;
        //double margin_penalty = 10.0; 
        //State state(nx, ny, 0, map, margin, margin_penalty, cell_num_x_);
        //State state(nx, ny, 0, map); 
 
        // 障害物なしの場合、ノードを追加
        if(isInsideMap(nx, ny) && !isObstacle(nx, ny, map)) {
          Node node(nx, ny);
          adjacent_nodes.push_back(node);
        }

        //ROS_INFO("Added neighbor at (%d, %d)", nx, ny);
        //ROS_INFO("Found %lu adjacent nodes", adjacent_nodes.size());
    }
    //ROS_INFO("Found %lu adjacent nodes", adjacent_nodes.size());
    return adjacent_nodes;

}

// 2つのノード間の移動コストを取得
double Astar_ValueIterator::getMoveCost(const Node& from, const Node& to) 
{
  // マンハッタン距離
  return abs(from.x - to.x) + abs(from.y - to.y);
}

void Astar_ValueIterator::cellDelta(double x, double y, double t, int &ix, int &iy, int &it)
{
	ix = (int)floor(fabs(x) / xy_resolution_);
	if(x < 0.0)
		ix = -ix-1;
	iy = (int)floor(fabs(y) / xy_resolution_);
	if(y < 0.0)
		iy = -iy-1;

	it = (int)floor(t / t_resolution_);
}


// 障害物か判定 

// A*で計算されたパスに対して局所的な価値反復を実行


// A*で計算されたパスに対して価値反復を行うメソッド




} // namespace value_iteration