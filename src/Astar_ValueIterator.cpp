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

  astar_xy_range_ = 1.0;
	astar_ixy_range_ = (int)(astar_xy_range_/xy_resolution_);
	astar_ix_min_ = 0;
	astar_iy_min_ = 0;
	astar_ix_max_ = astar_ixy_range_*2;
	astar_iy_max_ = astar_ixy_range_*2;
  //setState(map, safety_radius, safety_radius_penalty);
	//setStateTransition();
  //setSweepOrders();

  //states_.clear();
	//int margin = (int)ceil(safety_radius/xy_resolution_);
  //setState(map, safety_radius, safety_radius_penalty);
    
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
  //std::vector<Node> path;
  // 開始ノード
  int start_x = -3.0;
  int start_y = 1.0;
  Node start_node(start_x, start_y);
  ROS_INFO("Start node: (%d, %d)", start_node.x, start_node.y);

  // ゴールノード
  //goal_x_ = goal.position.x;
  //goal_y_ = goal.position.y;
  //ROS_INFO("Goal: (%f, %f)", goal_x_, goal_y_);

  //int goal_x_int = (int)goal_x_; 
  //int goal_y_int = (int)goal_y_;
  int goal_x_int = (int)goal.position.x; 
  int goal_y_int = (int)goal.position.y;
  Node goal_node(goal_x_int, goal_y_int);

  //Node goal_node;
  //goal_node.x = goal_x_;
  //goal_node.y = goal_y_;
  ROS_INFO("Goal node: (%d, %d)",  goal_node.x,  goal_node.y);

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

    ROS_INFO("Current node: (%d, %d, %f)", current_node.x, current_node.y, current_node.f);

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

  //ROS_INFO("Path found with %lu nodes", path.size());

  astarPath = path;

  //ROS_INFO("A* Path found with %lu nodes", astarPath.size());

  //for(auto& pose : path) {

  //  geometry_msgs::Pose node;
  //  node.position.x = pose.position.x;
  //  node.position.y = pose.position.y;

    // nodeをastarPathに追加 
  //  astarPath.push_back(node);

  //}

  // astarPathの内容を出力
  //for(auto& node : astarPath) {
  //  ROS_INFO("- x:%f, y:%f", node.position.x, node.position.y); 
  //}

  //std::vector<int> stateIndexPath = convertAstarPathToStateIndex(astarPath);

  //valueIterationAstarPath(stateIndexPath);

  return astarPath;

}

// goalをセット
//void Astar_ValueIterator::setGoal(double goal_x, double goal_y)
//{
//	goal_x_ = goal_x;
//	goal_y_ = goal_y;

//  ROS_INFO("GOAL: %f, %f", goal_x_, goal_y_);

	//ROS_INFO("GOAL: %f, %f", goal_x_, goal_y_);
  // ここで初期コスト設定
  //setStateValues();

  // ステータスやスレッド情報のクリア
//  thread_status_.clear();
//  setStateValues();
//  status_ = "calculating";

//}

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
  //std::vector<Node> path;
  //Node current = goal;
  geometry_msgs::Pose current;
  current.position.x = goal.x;
  current.position.y = goal.y;
  // closedリストを後ろからたどる
  for (int i = closed.size() - 1; i >= 0; i--) {

    // currentをpathに追加
    path.insert(path.begin(), current);

    // closedのノードを次のcurrentに設定 
    current.position.x = closed[i].x;
    current.position.y = closed[i].y;
    //ROS_INFO("Adding current node to path: (%f, %f)", current.position.x, current.position.y);
  }
  // Add the start node to the path
  //ROS_INFO("Adding start node to path: (%d, %d)", current.x, current.y);
  geometry_msgs::Pose start_pose;
  start_pose.position.x = current.position.x;
  start_pose.position.y = current.position.y;
  path.insert(path.begin(), start_pose);

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
    ROS_INFO("Found %lu adjacent nodes", adjacent_nodes.size());
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

// A*で計算されたパスに対して局所的な価値反復を実行

void Astar_ValueIterator::valueIterationAstarPathWorker(int id)
{
  setWindowsOnAstarPath();

	while(status_ == "canceled" or status_ == "goal"){
		ROS_INFO("STATUS PROBLEM: %s", status_.c_str());
		status_ = "executing";
	}

	while(status_ != "canceled" and status_ != "goal"){
		astarValueIterationLoop();
	}
}

void Astar_ValueIterator::astarValueIterationLoop(void)
{
	for(int iix=astar_ix_min_;iix<=astar_ix_max_;iix++){
		for(int iiy=astar_iy_min_;iiy<=astar_iy_max_;iiy++){
			for(int iit=0;iit<cell_num_t_;iit++){
				int i = toIndex(iix, iiy, iit);
				valueIterationAstarPath(states_[i]);
			}
		}
	}
}

uint64_t Astar_ValueIterator::valueIterationAstarPath(State &s)
{
	if((not s.free_) or s.final_state_)
    //ROS_INFO("False");
		return 0;

	uint64_t min_cost = ValueIterator::max_cost_;
	Action *min_action = NULL;
	for(auto &a : actions_){
		int64_t c = actionCostAstar(s, a);
		if(c < min_cost){
			min_cost = c;
			min_action = &a;
		}
	}

	int64_t delta = min_cost - s.total_cost_;
	s.total_cost_ = min_cost;
	s.optimal_action_ = min_action;
  ROS_INFO("After update: cost=%lu", s.total_cost_);

	return delta > 0 ? delta : -delta;
}

bool Astar_ValueIterator::inAstarArea(int x, int y) 
{
  // astarウィンドウの範囲内かチェック
  return (x >= astar_ix_min_ && x <= astar_ix_max_ && y >= astar_iy_min_ && y <= astar_iy_max_);

}

uint64_t Astar_ValueIterator::actionCostAstar(State &s, Action &a)
{
	uint64_t cost = 0;
	for(auto &tran : a._state_transitions[s.it_]){
		int ix = s.ix_ + tran._dix;
		if(ix < 0 or ix >= cell_num_x_)
			return max_cost_;

		int iy = s.iy_ + tran._diy;
		if(iy < 0 or iy >= cell_num_y_)
			return max_cost_;

		int it = (tran._dit + cell_num_t_)%cell_num_t_;

		auto &after_s = states_[toIndex(ix, iy, it)];
		if(not after_s.free_)
			return max_cost_;

		cost += ( after_s.total_cost_ + after_s.penalty_ + after_s.local_penalty_ ) * tran._prob;
	}

	return cost >> prob_base_bit_;
}

void Astar_ValueIterator::setWindowsOnAstarPath() 
{
  for (geometry_msgs::Pose pose : astarPath) {
    //ROS_INFO("START MAP");
    // poseからx,yを取得してNodeに変換
    Node node(pose.position.x, pose.position.y);
    //ROS_INFO("START MAP");
    setAstarWindow(node);
    
    // マップ作成

  }
  //ROS_INFO("GET MAP");

}

// A*経路上のnodeを中心にウィンドウを設定
void Astar_ValueIterator::setAstarWindow(const Node& center_node) 
{
  //ROS_INFO("START MAP");
  double resolution = xy_resolution_;

  int index_x = (int)(center_node.x / resolution);  
  int index_y = (int)(center_node.y / resolution);
  // 1. 中心ノードのインデックスを取得
  int center_x = index_x;
  int center_y = index_y;

  //ROS_INFO("center_y: %d", center_y);

  // 2. ウィンドウサイズからオフセットを計算
  int offset = astar_xy_range_;
  
  astar_ix_min_ = center_x - offset;
  astar_ix_max_ = center_x + offset;

  astar_iy_min_ = center_y - offset; 
  astar_iy_max_ = center_y + offset;

  // 3. マップサイズの調整
  astar_ix_min_ = std::max(astar_ix_min_, 0);
  astar_ix_max_ = std::min(astar_ix_max_, cell_num_x_-1);

  astar_iy_min_ = std::max(astar_iy_min_, 0);
  astar_iy_max_ = std::min(astar_iy_max_, cell_num_y_-1);

}

void Astar_ValueIterator::makeAstarValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold,
		double x, double y, double yaw_rad)
{
  // ウィンドウサイズから幅と高さを設定
  int width = astar_ix_max_ - astar_ix_min_ + 1;
  int height = astar_iy_max_ - astar_iy_min_ + 1;

  // マップ情報設定
  map.header.stamp = ros::Time::now();
  map.header.frame_id = "map";

  map.info.width = width;
  map.info.height = height;
  map.info.resolution = xy_resolution_;
  
  map.info.origin.position.x = x - astar_ixy_range_;
  map.info.origin.position.y = y - astar_ixy_range_;
  map.info.origin.orientation = map_origin_quat_;

  //ROS_INFO("astar_ix_max_: %d", width);
  //ROS_INFO("astar_iy_max_: %d", height);

  // データサイズを幅x高さに設定
  map.data.resize(width * height);


        int it = (int)floor( ( ((int)(yaw_rad/M_PI*180) + 360*100)%360 )/t_resolution_ );

	for(int y = 0; y < height; y++)
    //ROS_INFO("y: %d", y);
		for(int x = 0; x < width; x++){
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

} // namespace value_iteration