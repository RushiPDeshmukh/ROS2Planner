
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "graph_interfaces/srv/get_path.hpp"
#include <vector>
#include <queue>


using namespace std;

class BFSPlannerNode : public rclcpp::Node
{
  public:
    BFSPlannerNode()
    : Node("BFSPlannerNode")
    {
      server_ = this->create_service<graph_interfaces::srv::GetPath>("getPlan",
      bind(&BFSPlannerNode::service_handler,this,std::placeholders::_1,std::placeholders::_2));
      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node created successfully!");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for requests...");  
    }

  private:
    rclcpp::Service<graph_interfaces::srv::GetPath>::SharedPtr server_;
    long int start {0}; 
    long int goal {0}; 
    long int nodes {0};
    vector<vector <int>> adjMatrix;
    vector< vector<int>> adjList;

    void service_handler(const std::shared_ptr<graph_interfaces::srv::GetPath::Request> request,
        std::shared_ptr<graph_interfaces::srv::GetPath::Response> response)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
        if (request->target.start == request->target.goal){
            response->path = vector<long int> {request->target.start};
            return;
        }
        this->start = request->target.start;
        this->goal = request->target.goal;
        this->nodes = request->nodes;
        // Creating an empty zeros 2d vector of size nodes x nodes
        vector<int> temp (nodes,0);
        for(int i = 0;i<nodes;i++){
            this->adjMatrix.push_back(temp);
        }
        // Creating Adjacency Matrix 2d vector of size nodes x nodes
        int curr_index = 0;
        int ii = 0;
        int jj = 0;
        for(auto i:request->data){
            if(i==1){
            ii = curr_index%nodes;
            jj = curr_index/nodes;
            this->adjMatrix[ii][jj]=1;
            }
            curr_index++;
        }
        this->convert_adjList();

        response->path = this->bfs_planner();
        
        return;
    }


    void convert_adjList()
    {
    for(int i=0;i<this->nodes;i++){
        vector<int> temp;
        for(int j=0;j<this->nodes;j++){
        if(this->adjMatrix[i][j]==1){
            temp.push_back(j);
        }
        }
        this->adjList.push_back(temp);
    }
    return;
    }

    vector<long int> bfs_planner()
    {
    queue<int> q1;
    vector<long int> path;
    vector<int> parent(this->adjList.size(),-1);
    vector<int> visited(this->adjList.size(),-1);
    q1.push(this->start);
    int curr_node = -1;
    while(!q1.empty()){
        curr_node = q1.front();
        q1.pop();
        visited[curr_node] = 1;
        if(curr_node==this->goal){
        break;
        }
        else{
        for(auto next_node:this->adjList[curr_node]){
            if(visited[next_node]==-1){
            q1.push(next_node);
            parent[next_node]=curr_node;
            }
        }
        }
    }
    int temp = this->goal;
    path.push_back(this->goal);
    while(temp!=start){
        path.push_back(parent[temp]);
        temp= parent[temp];
    }
    reverse(path.begin(),path.end());
    return path;

    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BFSPlannerNode>());
  rclcpp::shutdown();
  return 0;
}