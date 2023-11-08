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
    
	astar_ix_max_ = 0;
	astar_iy_min_ = 0;
	astar_ix_max_ = 0;
	astar_iy_max_ = 0;
    
}

double Astar_ValueIterator::calculateHeuristic(const Node& node, const geometry_msgs::Pose& goal) 
{
    // ユークリッド距離を計算する例
    double dx = node.x - goal.position.x;
    double dy = node.y - goal.position.y;
    return sqrt(dx * dx + dy * dy);
}

std::vector<geometry_msgs::Pose> Astar_ValueIterator::calculateAStarPath(nav_msgs::OccupancyGrid &map, geometry_msgs::Pose start, geometry_msgs::Pose goal)
{
    // A*アルゴリズムの実装
    ROS_INFO("A* Algorithm Started");
    //ROS_INFO("SET ASTAR");
    // スタートノードを作成してオープンリストに追加
    Node startNode(start.position.x, start.position.y);
    startNode.g = 0;
    startNode.h = calculateHeuristic(startNode, goal);
    startNode.f = startNode.g + startNode.h;
    openList.push_back(startNode);

    double _delta_fw = 2.0;  // ローカル変数として宣言
    double _delta_rot = 20.0;
    int id_ = 0.0;

    //Action myAction("ActionName", _delta_fw, _delta_rot, id_);

    while (!openList.empty()) 
    {
        // オープンリストからf値が最小のノードを取得
        Node currentNode = getLowestFNode(openList);
        ROS_INFO("SET ASTAR");
        // ゴールに到達したら経路を構築して返す
        if (reachedGoal(currentNode, goal)) 
        {
            ROS_INFO("Goal Reached. Constructing Path...");
            std::vector<geometry_msgs::Pose> path = buildPath(currentNode);
            ROS_INFO("A* PATH CALCULATION FINISHED");
            return path;
        }

        // オープンリストから取り出したノードをクローズドリストに移動
        openList.erase(std::remove(openList.begin(), openList.end(), currentNode), openList.end());
        closedList.push_back(currentNode);

        // 隣接ノードを展開
        std::vector<Node> neighbors = getNeighbors(currentNode, map);

        ROS_INFO("Expanding %zu neighbors.", neighbors.size());

        for (Node &neighbor : neighbors) 
        {
            ROS_INFO("aaaa ASTAR");
            // クローズドリストにあるノードはスキップ
            if (containsNode(closedList, neighbor)) 
            {
                continue;
            }

            // アクション情報を生成
            Action myAction("ActionName", _delta_fw, _delta_rot, id_);
            //Action myAction("ActionName", myAction._delta_fw, myAction._delta_rot, myAction.id_);
            //Action myAction("ActionName", actions);

            // オープンリストにないノードまたはより低いコストであるノードを更新
            int tentativeG = currentNode.g + calculateDistance(currentNode, neighbor);

            //int64_t cost = calculateActionCostForAStarState(neighbor, myAction);
            int64_t cost = calculateActionCostForAStarState(neighbor.toState(), myAction, map);

            if (!containsNode(openList, neighbor) || tentativeG < neighbor.g) 
            {
                neighbor.g = tentativeG;
                neighbor.h = calculateHeuristic(neighbor, goal);
                neighbor.f = neighbor.g + neighbor.h;
                neighbor.parent = &currentNode;

                if (!containsNode(openList, neighbor)) 
                {
                    openList.push_back(neighbor);
                }
            }
        }
    }

    return std::vector<geometry_msgs::Pose>();

}

// A*で計算されたパスに対して局所的な価値反復を実行
void Astar_ValueIterator::astarValueIterationLoop(nav_msgs::OccupancyGrid &map) 
{
    ROS_INFO("SET ASTAR LOCAL");
    for (int iix = astar_ix_min_; iix <= astar_ix_max_; iix++) {
        for (int iiy = astar_iy_min_; iiy <= astar_iy_max_; iiy++) {
            for (int iit = 0; iit < cell_num_t_; iit++) {
                // 状態のインデックスを計算
                int index = toIndex(iix, iiy, iit);

                // A*で計算されたパスに対して価値反復を実行
                performValueIterationForAStarPath(states_[index], map);
            }
        }
    }
}


// A*で計算されたパスに対して価値反復を行うメソッド
void Astar_ValueIterator::performValueIterationForAStarPath(State& state, nav_msgs::OccupancyGrid& map) 
{
    if (!state.free_ || state.final_state_) {
        return;
    }

    uint64_t min_cost = ValueIterator::max_cost_;
    Action* min_action = nullptr;

    // 価値反復のためのアクション選択とコスト計算を実行
    for (auto& action : actions_) {
        int64_t cost = calculateActionCostForAStarState(state, action, map);

        if (cost < min_cost) {
            min_cost = cost;
            min_action = &action;
        }
    }

    int64_t delta = min_cost - state.total_cost_;
    state.total_cost_ = min_cost;
    state.optimal_action_ = min_action;

    if (delta > 0) {
        delta = -delta;
    }
//    return delta > 0 ? delta : -delta;
}

void Astar_ValueIterator::astarValueIterationWorker(int id)
{
	while(thread_status_[id]._delta < 0.1 or status_ == "canceled" or status_ == "goal"){
		ROS_INFO("STATUS PROBLEM: %s", status_.c_str());
		status_ = "executing";
	}

	while(thread_status_[id]._delta < 0.1 and status_ != "canceled" and status_ != "goal"){
		astarValueIterationLoop(map_);
	}
}

Action *Astar_ValueIterator::posToAction(double x, double y, double t_rad)
{
        int ix = (int)floor( (x - map_origin_x_)/xy_resolution_ );
        int iy = (int)floor( (y - map_origin_y_)/xy_resolution_ );

        int t = (int)(180 * t_rad / M_PI);
        int it = (int)floor( ( (t + 360*100)%360 )/t_resolution_ );
	int index = toIndex(ix, iy, it);

	if(states_[index].final_state_){
		status_ = "goal";
		return NULL;
	}else if(states_[index].optimal_action_ != NULL){
		ROS_INFO("COST TO GO: %f", (double)states_[index].total_cost_/ValueIterator::prob_base_);
		return states_[index].optimal_action_;
	}

	return NULL;

}

Node Astar_ValueIterator::getLowestFNode(const std::vector<Node>& openList) 
{
    // 最初のノードを最小と仮定
    Node lowestFNode = openList[0];

    // オープンリスト内の各ノードを検討し、f値が最小のノードを見つける
    for (const Node& node : openList) {
        if (node.f < lowestFNode.f) {
            lowestFNode = node;
        }
    }

    return lowestFNode;
}

bool Astar_ValueIterator::reachedGoal(const Node& node, const geometry_msgs::Pose& goal) 
{
    // ノードの位置と目標地点の位置のユークリッド距離を計算
    double dx = node.x - goal.position.x;
    double dy = node.y - goal.position.y;
    double distance = sqrt(dx * dx + dy * dy);

    // 到達判定：ユークリッド距離が一定の閾値未満であれば到達したとみなす
    double threshold = 0.1; 
    return distance <= threshold;
}

std::vector<Node> Astar_ValueIterator::getNeighbors(const Node& node, nav_msgs::OccupancyGrid &map) 
{
    std::vector<Node> neighbors;

    // 8つの方向に対して隣接ノードをチェック
    int dx[] = {1, -1, 0, 0, 1, -1, 1, -1}; // 水平方向の変化
    int dy[] = {0, 0, 1, -1, 1, 1, -1, -1}; // 垂直方向の変化

    for (int i = 0; i < 8; i++) {
        int new_x = node.x + dx[i];
        int new_y = node.y + dy[i];
        ROS_INFO("Added neighbor at (%d, %d)", new_x, new_y);
        // マップの範囲内かどうかを確認
        if (new_x >= 0 && new_x < map.info.width && new_y >= 0 && new_y < map.info.height) {
            int index = new_x + new_y * map.info.width;
            // マップの障害物情報を確認
            if (map.data[index] != 100) {
                // 障害物でない場合、新しいノードを作成して neighbors に追加
                Node neighbor(new_x, new_y);
                neighbors.push_back(neighbor);
            }
        }
    }

    return neighbors;
}

bool Astar_ValueIterator::containsNode(const std::vector<Node>& nodeList, const Node& node)
{
    for (const Node& n : nodeList) {
        // ノードの比較方法。ノードの座標が同じかどうかを確認。
        if (n.x == node.x && n.y == node.y) {
            return true; // 同じノードが見つかった
        }
    }
    return false; // 同じノードが見つからなかった
}

double Astar_ValueIterator::calculateDistance(const Node& from, const Node& to) 
{
    // ユークリッド距離を計算する
    double dx = from.x - to.x;
    double dy = from.y - to.y;
    return sqrt(dx * dx + dy * dy);
}

int64_t Astar_ValueIterator::calculateActionCostForAStarState(const State& state, const Action& action, const nav_msgs::OccupancyGrid &map) 
{
  // 移動コストと回転コストを算出
  double move_cost = calculateMoveCost(state, action, map);
  double rotate_cost = calculateRotateCost(action);

  // 総コストを計算
  int64_t total_cost = calculateTotalCost(move_cost, rotate_cost);

  return total_cost;
}

// stateからマップ位置のコストを取得
double Astar_ValueIterator::getMapCost(const State& state, const nav_msgs::OccupancyGrid &map) 
{
  // 状態からマップインデックスを計算
  int index = state.x + state.y * cell_num_x_;
  
  // マップからコストを取得
  double map_cost = map.data[index];

  return map_cost; 
}

// 移動コストを計算
double Astar_ValueIterator::calculateMoveCost(const State& state, const Action& action, const nav_msgs::OccupancyGrid& map) 
{
  // マップコスト * 移動距離 
  double move_cost = action.delta_distance * getMapCost(state, map);

  return move_cost;
}

// 回転コストを計算
double Astar_ValueIterator::calculateRotateCost(const Action& action) 
{
  // 回転角度 * 回転コスト定数
  double rotation_cost_param = 1.0;
  double rotate_cost = abs(action.delta_theta) * rotation_cost_param;

  return rotate_cost;
}

int64_t Astar_ValueIterator::calculateTotalCost(double move_cost, double rotate_cost) 
{
  // 移動コスト + 回転コスト
  int64_t total_cost = move_cost + rotate_cost;

  return total_cost;
}

std::vector<geometry_msgs::Pose> Astar_ValueIterator::buildPath(const Node& goalNode) 
{
  // 経路を格納する配列
  std::vector<geometry_msgs::Pose> path;

  // 現在のノードをゴールノードとする
  Node current = goalNode;

  // スタートノードに到達するまで、親ノードに沿って経路をたどる
  while (current.parent != nullptr) {

    // 現在のノードの座標からPoseメッセージを作成
    geometry_msgs::Pose pose;
    pose.position.x = current.x;
    pose.position.y = current.y;

    // 経路の頭に追加(逆順にたどる)
    path.insert(path.begin(), pose);

    // 親ノードを現在のノードとして更新
    current = *(current.parent); 
  }

  // 完成した経路配列を返す
  return path;
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

} // namespace value_iteration