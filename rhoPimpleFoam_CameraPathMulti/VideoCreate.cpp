#include <iostream>
#include <vector>
#include <limits>
#include <climits>
#include <set>
#include <stack>
#include <fstream>
#include <sstream>
#include <string>
#include <boost/filesystem.hpp>
#include <chrono>
#include <cmath>
#include <algorithm> // reverse
#include <cstdlib>   // atoi, atof

namespace fs = boost::filesystem;
using namespace std;

class Graph {
public:
    int V;
    vector<vector<pair<int, float>>> adj;

    Graph(int V) {
        this->V = V;
        adj.resize(V);
    }

    void addEdge(int u, int v, float weight) {
        adj[u].push_back(make_pair(v, weight));
    }

    void dijkstra(int start, vector<float>& dist, vector<int>& parent) {
        dist.assign(V, std::numeric_limits<float>::max());
        parent.assign(V, -1);
        dist[start] = 0.0f;

        set<pair<float, int>> s; // (distance, node)
        s.insert(make_pair(0.0f, start));

        while (!s.empty()) {
            int u = s.begin()->second;
            s.erase(s.begin());

            for (auto& neighbor : adj[u]) {
                int v = neighbor.first;
                float weight = neighbor.second;
                if (dist[u] + weight < dist[v]) {
                    if (dist[v] != std::numeric_limits<float>::max()) {
                        s.erase(make_pair(dist[v], v));
                    }
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    s.insert(make_pair(dist[v], v));
                }
            }
        }
    }

    stack<int> printPath(const vector<int>& parent, int node) {
        if (node == -1) return {};
        stack<int> path;
        int current = node;
        while (current != -1) {
            path.push(current);
            current = parent[current];
        }
        return path;
    }
};

std::vector<float> extractColumnFloat(const std::string& filename, size_t column_index) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return {};
    }

    std::vector<std::string> column_data;
    std::string line;
    std::getline(file, line); // header
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string cell;
        size_t current_index = 0;

        while (std::getline(ss, cell, ',')) {
            if (current_index == column_index) {
                column_data.push_back(cell);
                break;
            }
            ++current_index;
        }
    }

    file.close();

    std::vector<float> vec;
    vec.reserve(column_data.size());
    for (auto& value : column_data) {
        vec.push_back(std::stof(value));
    }
    return vec;
}

std::vector<std::string> extractColumn(const std::string& filename, size_t column_index) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return {};
    }

    std::vector<std::string> column_data;
    std::string line;
    std::getline(file, line); // header
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string cell;
        size_t current_index = 0;

        while (std::getline(ss, cell, ',')) {
            if (current_index == column_index) {
                column_data.push_back(cell);
                break;
            }
            ++current_index;
        }
    }

    file.close();

    return column_data;
}

std::vector<std::string> listFiles(const fs::path& directory) {
    std::vector<std::string> files;
    try {
        if (!fs::exists(directory) || !fs::is_directory(directory)) {
            std::cerr << "Invalid directory: " << directory << std::endl;
            return {};
        }

        for (const auto& entry : fs::directory_iterator(directory)) {
            if (fs::is_regular_file(entry)) {
                std::cout << "File: " << entry.path().filename().string() << std::endl;
                files.push_back(entry.path().filename().string());
            } else if (fs::is_directory(entry)) {
                std::cout << "Directory: " << entry.path().filename().string() << std::endl;
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    }
    return files;
}

std::vector<int> getFilenameDetail(std::string filename){
    std::vector<int> file;
    size_t firstUnderscore = filename.find('_');
    size_t secondUnderscore = filename.find('_', firstUnderscore + 1);
    size_t thirdUnderscore = filename.find('_', secondUnderscore + 1);
    size_t fourthUnderscore = filename.find('_', thirdUnderscore + 1);
    size_t periodScore = filename.find('.');

    int time = atoi((filename.substr(firstUnderscore + 1, secondUnderscore - firstUnderscore - 1)).c_str());
    int candidate = atoi((filename.substr(secondUnderscore + 1, thirdUnderscore - secondUnderscore - 1)).c_str());
    int zoomLevel = atoi((filename.substr(thirdUnderscore + 1, fourthUnderscore - thirdUnderscore - 1)).c_str());
    int route = atoi((filename.substr(fourthUnderscore + 1, periodScore - fourthUnderscore - 1)).c_str());

    file.push_back(time);
    file.push_back(candidate);
    file.push_back(zoomLevel);
    file.push_back(route);
    return file;
}

bool getRouteImage(int time_from, int time_to, int candidate_num, int from, int to, std::string filename ){
    int routeNum = (from % candidate_num) * candidate_num + (to % candidate_num);

    size_t firstUnderscore = filename.find('_');
    size_t secondUnderscore = filename.find('_', firstUnderscore + 1);
    size_t thirdUnderscore = filename.find('_', secondUnderscore + 1);
    size_t fourthUnderscore = filename.find('_', thirdUnderscore + 1);
    size_t periodScore = filename.find('.');

    int time = atoi((filename.substr(firstUnderscore + 1, secondUnderscore - firstUnderscore - 1)).c_str());
    int route = atoi((filename.substr(fourthUnderscore + 1, periodScore - fourthUnderscore - 1)).c_str());

    if (time > time_from && time < time_to) {
        if (routeNum == route) {
            return true;
        }
    }
    return false;
}

/* =========================
 * NEW: cumulative distance output
 * =========================
 * Output cumulative focus distance (pFP) and camera distance (pCP)
 * along the decided Dijkstra path.
 *
 * Assumption:
 * pFP and pCP are stored as blocks per timestep:
 *  block(t) size = candidate_num * candidate_num
 *  idx = t_from * block_size + candidate_num * to_c + from_c
 * where t_from is 0-based timestep index (fromNode / candidate_num).
 */
bool outputCumulativeDistancesCSV(
    const std::string& out_csv_path,
    const std::vector<int>& path_nodes,        // start -> goal order
    const std::vector<std::string>& fn,        // filenames per node
    const std::vector<float>& pFP,
    const std::vector<float>& pCP,
    int candidate_num
) {
    if (path_nodes.size() < 2) {
        std::cerr << "[WARN] path_nodes too short. Nothing to output.\n";
        return false;
    }

    std::ofstream ofs(out_csv_path);
    if (!ofs.is_open()) {
        std::cerr << "[ERROR] Failed to open output csv: " << out_csv_path << "\n";
        return false;
    }

    ofs << "k,fromNode,toNode,fromTime,toTime,from_c,to_c,focus_dist,camera_dist,cum_focus_dist,cum_camera_dist\n";

    const size_t block_size = (size_t)candidate_num * (size_t)candidate_num;
    float cum_fp = 0.0f;
    float cum_cp = 0.0f;

    for (size_t k = 0; k + 1 < path_nodes.size(); ++k) {
        int fromNode = path_nodes[k];
        int toNode   = path_nodes[k + 1];

        int t_from = fromNode / candidate_num; // 0-based timestep index
        int from_c = fromNode % candidate_num;
        int to_c   = toNode   % candidate_num;

        size_t idx = (size_t)t_from * block_size + (size_t)candidate_num * (size_t)to_c + (size_t)from_c;

        if (idx >= pFP.size() || idx >= pCP.size()) {
            std::cerr << "[ERROR] idx out of range: idx=" << idx
                      << " pFP.size=" << pFP.size()
                      << " pCP.size=" << pCP.size() << "\n";
            ofs.close();
            return false;
        }

        float fp = pFP[idx];
        float cp = pCP[idx];

        cum_fp += fp;
        cum_cp += cp;

        int fromTime = -1;
        int toTime   = -1;
        if ((size_t)fromNode < fn.size() && (size_t)toNode < fn.size()) {
            auto a = getFilenameDetail(fn[fromNode]);
            auto b = getFilenameDetail(fn[toNode]);
            fromTime = a.empty() ? -1 : a[0];
            toTime   = b.empty() ? -1 : b[0];
        }

        ofs << k << ","
            << fromNode << "," << toNode << ","
            << fromTime << "," << toTime << ","
            << from_c << "," << to_c << ","
            << fp << "," << cp << ","
            << cum_fp << "," << cum_cp << "\n";
    }

    ofs.close();
    std::cerr << "[INFO] Wrote cumulative distances: " << out_csv_path << "\n";
    return true;
}

int main(int argc, char *argv[]) {
    chrono::system_clock::time_point start, end;

    std::vector<std::string> fn;
    std::vector<float> en;
    std::vector<float> pFP;
    std::vector<float> pCP;
    std::string cell;
    stack<int> path_stack;

    if (argc < 6) {
        std::cerr << "Usage: " << argv[0]
                  << " <candidate_num> <first_file> <entropy_ratio> <focus_ratio> <camera_ratio>\n";
        return 1;
    }

    auto candidate_num = atoi(argv[1]);
    auto first_file = atoi(argv[2]);
    auto entropy_ratio = atof(argv[3]);
    auto focus_path_length_ratio = atof(argv[4]);
    auto camera_path_length_ratio = atof(argv[5]);

    std::ifstream file("../realistic-s3/Output/output_video_params.csv");
    if (!file.is_open()) {
        std::cerr << "ファイルを開けませんでした。" << std::endl;
        return 1;
    }

    fn = extractColumn("../realistic-s3/Output/output_video_params.csv", 0);
    en = extractColumnFloat("../realistic-s3/Output/output_video_params.csv", 1);

    for (auto& val : en) {
        val = 1.0f / val;
    }

    Graph g((int)fn.size());

    std::string line;
    std::getline(file, line); // header

    // Read pFP/pCP columns from each row
    while (std::getline(file, line)) {
        std::istringstream i_stream(line);
        int n = 0;
        while (getline(i_stream, cell, ',')) {
            auto a = atof(cell.c_str());
            if (n > 1 && 2 + candidate_num > n) pFP.push_back((float)a);
            else if (n > 1 + candidate_num && 2 * (candidate_num) + 2 > n) pCP.push_back((float)a);
            n++;
        }
    }

    // Reset stream position (as in your original code)
    file.clear();
    file.seekg(0);

    for (size_t i = 0; i < (size_t)candidate_num + 1; i++) {
        std::getline(file, line);
    }

    int check = 1;
    for (size_t i = 0; i < fn.size(); i++) {
        float sum_en = 0.0f;
        auto path_base = (size_t)candidate_num * (size_t)candidate_num * (size_t)(check - 1);
        float sum_pFP = 0.0f;
        float sum_pCP = 0.0f;

        for (size_t j = 0; j < (size_t)candidate_num; j++) {
            sum_en += en[(size_t)check * (size_t)candidate_num + j];
            sum_pFP += pFP[path_base + (size_t)candidate_num * j + (i % (size_t)candidate_num)];
            sum_pCP += pCP[path_base + (size_t)candidate_num * j + (i % (size_t)candidate_num)];
        }

        // (Optional) guard to avoid 0-division crash
        const float eps = 1e-12f;
        if (sum_en  < eps) sum_en  = eps;
        if (sum_pFP < eps) sum_pFP = eps;
        if (sum_pCP < eps) sum_pCP = eps;

        for (size_t j = 0; j < (size_t)candidate_num; j++) {
            auto weight =
                (float)(entropy_ratio * en[(size_t)check * (size_t)candidate_num + j] / sum_en) +
                (float)(focus_path_length_ratio * pFP[path_base + (size_t)candidate_num * j + (i % (size_t)candidate_num)] / sum_pFP) +
                (float)(camera_path_length_ratio * pCP[path_base + (size_t)candidate_num * j + (i % (size_t)candidate_num)] / sum_pCP);

            g.addEdge((int)i, (int)((size_t)check * (size_t)candidate_num + j), weight);
        }

        if (i % (size_t)candidate_num == (size_t)candidate_num - 1) check++;
    }

    vector<int> goals;
    for (int i = 0; i < candidate_num; i++) {
        goals.push_back((int)fn.size() - i - 1);
    }

    vector<float> dist;
    vector<int> parent;

    start = chrono::system_clock::now();
    g.dijkstra(first_file, dist, parent);
    end = chrono::system_clock::now();

    double time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
    printf("time %lf[ms]\n", time);

    float minDistance = std::numeric_limits<float>::max();
    int bestGoal = -1;
    for (int goal : goals) {
        if (goal >= 0 && goal < (int)dist.size() && dist[goal] < minDistance) {
            minDistance = dist[goal];
            bestGoal = goal;
        }
    }

    if (bestGoal != -1) {
        path_stack = g.printPath(parent, bestGoal);
    } else {
        cout << "ゴールノードへの経路が見つかりません。" << endl;
        return 1;
    }

    // Convert stack path to vector in time order (start -> goal)
    vector<int> path_nodes;
    while (!path_stack.empty()) {
        path_nodes.push_back(path_stack.top());
        path_stack.pop();
    }
    // If needed, uncomment this if your order is reversed in practice:
    // reverse(path_nodes.begin(), path_nodes.end());

    // Prepare output dirs and copy images as before
    std::string newDirPath = "../realistic-s3/ex_Output/";
    std::string sourceImagePath;

    fs::create_directory(newDirPath);
    fs::create_directory(newDirPath + "Output/");
    std::string destImagePath = newDirPath + "Output/";

    int fromImageindex = 0;
    int count = 0;

    start = chrono::system_clock::now();
    for (size_t pi = 0; pi < path_nodes.size(); ++pi) {
        int node = path_nodes[pi];

        sourceImagePath = "../realistic-s3/" + fn[node];
        destImagePath   = "../realistic-s3/ex_Output/" + fn[node];

        fs::copy_file(sourceImagePath, destImagePath, fs::copy_option::overwrite_if_exists);

        auto fromTime = getFilenameDetail(fn[fromImageindex]);
        auto toTime   = getFilenameDetail(fn[node]);

        if (count > 0) {
            for (const auto& entry : fs::directory_iterator("../realistic-s3/Output")) {
                if (getRouteImage(*fromTime.begin(), *toTime.begin(), candidate_num, fromImageindex, node, entry.path().filename().c_str())) {
                    auto f = "../realistic-s3/Output/" + entry.path().filename().string();
                    auto t = "../realistic-s3/ex_Output/Output/" + entry.path().filename().string();
                    fs::copy_file(f, t, fs::copy_option::overwrite_if_exists);
                }
            }
        }

        fromImageindex = node;
        count++;
    }
    end = chrono::system_clock::now();
    time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
    printf("time2 %lf[ms]\n", time);

    // NEW: Output cumulative distances along decided path
    outputCumulativeDistancesCSV(
        "../realistic-s3/ex_Output/path_cumulative_distances.csv",
        path_nodes,
        fn,
        pFP,
        pCP,
        candidate_num
    );

    return 0;
}
