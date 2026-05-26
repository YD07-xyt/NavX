// ------------------ 主程序 ------------------
int main(){
    using namespace std::chrono;
    auto t0 = high_resolution_clock::now();

    int rows = 60, cols = 80;
    vector<vector<int>> grid(rows, vector<int>(cols, 0));
    for(int r=0;r<40;++r) for(int c=15;c<19;++c) grid[r][c] = 50;
    for(int r=20;r<60;++r) for(int c=30;c<33;++c) grid[r][c] = 50;
    for(int r=40;r<43;++r) for(int c=40;c<60;++c) grid[r][c] = 50;

    pair<int,int> start = {0,0};
    pair<int,int> goal  = {79,59};

    // --------------------- A* ---------------------
    auto path_Astar = astar(start, goal, grid);
    if(path_Astar.empty())
        {cerr<<"No path found by A*"<<endl; return -1; }
    cout<<"Original path length: "<<path_Astar.size()<<"\n";
    auto t1 = high_resolution_clock::now();
    cout<<"A* time: "<<duration<double>(t1-t0).count()<<"s\n";



    
    // --------------------- Simplified ---------------------
    // 转换为vector<Eigen::Vector2d>
    vector<Eigen::Vector2d> path;
    for (auto &p : path_Astar)
        path.emplace_back(p.first, p.second);

    auto simplified = simplify_path(path, 30.0, 1);
    cout<<"Simplified length: "<<simplified.size()<<"\n";

    // --------------------- build corridor rectangles ---------------------
    double max_width = 7; double extend = 8;
    vector<pair<double,double>> path_xy;
    for(auto &p: simplified) 
        path_xy.emplace_back(p.first, p.second);

    // 走廊初始膨胀
    // 注意：当四个顶点只有一个被裁剪时会生成梯形
    auto rects = convex_corridor(path_xy, rows, cols, max_width, extend);
    // for (const auto& rect : rects) {
    //     for (const double& value : rect) {
    //         std::cout << value << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // convert and refine by splitting
    auto corridors = corridor_generator_optimized(path_xy, rects, grid, max_width);

    auto t2 = high_resolution_clock::now();
    cout<<"Corridor build time: "<<duration<double>(t2-t1).count()<<"s\n";

    // --------------------- Minimumsnap ---------------------
    int N = (int)simplified.size();
    vector<pair<int,int>> path_int;
    for(auto &p: simplified) 
        path_int.emplace_back(int(round(p.first)), int(round(p.second)));

    auto traj = minimum_snap_solver(corridors, grid, path_int, N, 2, "OSQP", 1.0);
    auto t3 = high_resolution_clock::now();  
    cout<<"Minimum-snap time: "<<duration<double>(t3-t2).count()<<"s\n";
    
    // --------------------- visualization ---------------------
    visualize_results(grid, path_Astar, simplified, corridors, traj, rects, start, goal, 12);
    
    // 输出轨迹点
    // cout<<"Optimized trajectory: \n";
    // for(auto &pt: traj) 
    //     cout<<pt.first<<", "<<pt.second<<"\n";

    return 0;
}
