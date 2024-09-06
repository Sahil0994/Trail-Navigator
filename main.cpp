#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <climits>
#include <algorithm>
#include <deque>

using namespace std;

// Class representing the metro graph
class Graph {
private:
    unordered_map<int, unordered_map<int, unordered_map<string, int> > > graph;
    unordered_map<string, int> station_numbers;
    unordered_map<int, string> station_names;
    int number_counter = 10; // Starting number for stations

public:
    // Method to add an edge to the graph
    void addEdge(string station1, string station2, int distance, int travel_time, int cost) {
        if (station_numbers.find(station1) == station_numbers.end()) {
            station_numbers[station1] = number_counter;
            station_names[number_counter] = station1;
            number_counter++;
        }
        if (station_numbers.find(station2) == station_numbers.end()) {
            station_numbers[station2] = number_counter;
            station_names[number_counter] = station2;
            number_counter++;
        }

        int s1 = station_numbers[station1];
        int s2 = station_numbers[station2];

        graph[s1][s2] = {{"distance", distance}, {"travel_time", travel_time}, {"cost", cost}};
        graph[s2][s1] = {{"distance", distance}, {"travel_time", travel_time}, {"cost", cost}};
    }

    // Get the station name based on the station number
    string getStationName(int station_number) {
        return station_names[station_number];
    }

    // Dijkstra's algorithm for finding the shortest path and distance
    pair<vector<int>, int> dijkstraShortestPath(int start_station, int end_station) {
        unordered_map<int, int> distances;
        unordered_map<int, int> previous;
        set<pair<int, int>> pq;  // {distance, station}

        for (auto& s : graph) {
            distances[s.first] = INT_MAX;
        }
        distances[start_station] = 0;
        pq.insert({0, start_station});

        while (!pq.empty()) {
            int current_station = pq.begin()->second;
            pq.erase(pq.begin());

            if (current_station == end_station) break;

            for (auto& neighbor : graph[current_station]) {
                int neighbor_station = neighbor.first;
                int new_distance = distances[current_station] + neighbor.second["distance"];

                if (new_distance < distances[neighbor_station]) {
                    pq.erase({distances[neighbor_station], neighbor_station});
                    distances[neighbor_station] = new_distance;
                    previous[neighbor_station] = current_station;
                    pq.insert({new_distance, neighbor_station});
                }
            }
        }

        vector<int> path;
        for (int at = end_station; at != start_station; at = previous[at]) {
            path.push_back(at);
        }
        path.push_back(start_station);
        reverse(path.begin(), path.end());

        return {path, distances[end_station]};
    }

    // Breadth-first search algorithm for finding the shortest path
    vector<int> bfsShortestPath(int start_station, int end_station) {
        deque<pair<int, vector<int>>> queue;
        set<int> visited;

        queue.push_back({start_station, {start_station}});

        while (!queue.empty()) {
            auto [current_station, path] = queue.front();
            queue.pop_front();

            visited.insert(current_station);

            for (auto& neighbor : graph[current_station]) {
                int neighbor_station = neighbor.first;
                if (visited.find(neighbor_station) == visited.end()) {
                    vector<int> new_path = path;
                    new_path.push_back(neighbor_station);
                    if (neighbor_station == end_station) return new_path;
                    queue.push_back({neighbor_station, new_path});
                }
            }
        }

        return {};
    }

    // Helper function for Prim's and Kruskal's algorithms
    int find_set(unordered_map<int, int>& parent, int station) {
        if (station != parent[station]) {
            parent[station] = find_set(parent, parent[station]);
        }
        return parent[station];
    }

    // Union function for Kruskal's algorithm
    void union_sets(unordered_map<int, int>& parent, int u, int v) {
        int root1 = find_set(parent, u);
        int root2 = find_set(parent, v);
        if (root1 != root2) {
            parent[root2] = root1;
        }
    }

    // Prim's algorithm for finding the minimum spanning tree and total travel time
    set<tuple<int, int, int> > primMinimumSpanningTree(int start_station, int end_station) {
        set<tuple<int, int, int> > mst;
        set<int> visited;
        priority_queue<tuple<int, int, int>, vector<tuple<int, int, int>>, greater<>> pq;

        pq.push({0, start_station, -1});

        while (!pq.empty()) {
            auto [cost, current_station, previous_station] = pq.top();
            pq.pop();

            if (visited.find(current_station) == visited.end()) {
                visited.insert(current_station);
                if (previous_station != -1) {
                    mst.insert({previous_station, current_station, cost});
                }

                for (auto& neighbor : graph[current_station]) {
                    if (visited.find(neighbor.first) == visited.end()) {
                        pq.push({neighbor.second["travel_time"], neighbor.first, current_station});
                    }
                }

                if (current_station == end_station) break;
            }
        }

        return mst;
    }

    // Kruskal's algorithm for finding the minimum spanning tree and total cost
    set<tuple<int, int, int> > kruskalMinimumSpanningTree(int start_station, int end_station) {
        vector<tuple<int, int, int> > edges;
        for (auto& s1 : graph) {
            for (auto& s2 : s1.second) {
                edges.push_back({s2.second["cost"], s1.first, s2.first});
            }
        }

        sort(edges.begin(), edges.end());

        set<tuple<int, int, int>> mst;
        unordered_map<int, int> parent;

        for (auto& s : graph) {
            parent[s.first] = s.first;
        }

        for (auto& [cost, s1, s2] : edges) {
            if (find_set(parent, s1) != find_set(parent, s2)) {
                mst.insert({s1, s2, cost});
                union_sets(parent, s1, s2);
            }

            if (find_set(parent, start_station) == find_set(parent, end_station)) break;
        }

        return mst;
    }

    // User interface for interacting with the metro graph
    void userInterface() {
        while (true) {
            cout << "------Welcome to Delhi Metro------\n";
            cout << "\nMenu:\n";
            cout << "1. Shortest path using BFS algorithm\n";
            cout << "2. Shortest distance using Dijkstra's Algorithm\n";
            cout << "3. Shortest Time using Prim's Algorithm\n";
            cout << "4. Minimum Cost using Kruskal's Algorithm\n";
            cout << "5. Exit\n";

            cout << "Station Numbers and Names:\n";
            for (auto& [name, number] : station_numbers) {
                cout << number << ": " << name << "\n";
            }

            int choice;
            cout << "Enter your choice (1/2/3/4/5): ";
            cin >> choice;

            int start_station, end_station;
            if (choice >= 1 && choice <= 4) {
                cout << "Enter the start station number: ";
                cin >> start_station;
                cout << "Enter the end station number: ";
                cin >> end_station;
            }

            if (choice == 1) {
                vector<int> shortest_path_bfs = bfsShortestPath(start_station, end_station);
                if (!shortest_path_bfs.empty()) {
                    cout << "Shortest path using BFS: ";
                    for (int station : shortest_path_bfs) {
                        cout << getStationName(station) << " -> ";
                    }
                    cout << "\n";
                } else {
                    cout << "No path found using BFS.\n";
                }

            } else if (choice == 2) {
                auto [shortest_path_dijkstra, distance_dijkstra] = dijkstraShortestPath(start_station, end_station);
                cout << "Shortest path using Dijkstra: ";
                for (int station : shortest_path_dijkstra) {
                    cout << getStationName(station) << " -> ";
                }
                cout << "\n";
                cout << "Distance: " << distance_dijkstra << " km\n";

            } else if (choice == 3) {
                auto mst_prim = primMinimumSpanningTree(start_station, end_station);
                int total_travel_time = 0;
                for (auto& edge : mst_prim) {
                    total_travel_time += get<2>(edge);
                }
                cout << "Minimum Spanning Tree using Prim's Algorithm (Shortest Time) total travel time: " << total_travel_time << " minutes\n";

            } else if (choice == 4) {
                auto mst_kruskal = kruskalMinimumSpanningTree(start_station, end_station);
                int total_cost = 0;
                for (auto& edge : mst_kruskal) {
                    total_cost += get<2>(edge);
                }
                cout << "Minimum Spanning Tree using Kruskal's Algorithm (Minimum Cost) total cost: " << total_cost << " rupees\n";

            } else if (choice == 5) {
                cout << "Exiting the program. Thank you!\n";
                break;
            } else {
                cout << "Invalid choice. Please enter a valid option.\n";
            }
        }
    }
};

int main() {
    Graph metro_graph;

    // Adding edges between stations along with distances, travel time, and cost
    vector<tuple<string, string, int, int, int>> edges = {
        {"Noida Sector 62~B", "Botanical Garden~B", 8, 15, 50},
        {"Botanical Garden~B", "Yamuna Bank~B", 10, 20, 60},
        {"Yamuna Bank~B", "Vaishali~B", 8, 18, 55},
        {"Yamuna Bank~B", "Rajiv Chowk~BY", 6, 12, 45},
        {"Rajiv Chowk~BY", "Moti Nagar~B", 9, 22, 70},
        {"Moti Nagar~B", "Janak Puri West~BO", 7, 16, 52},
        {"Janak Puri West~BO", "Dwarka Sector 21~B", 6, 14, 48},
        {"Huda City Center~Y", "Saket~Y", 15, 30, 90},
        {"Saket~Y", "AIIMS~Y", 6, 13, 42},
        {"AIIMS~Y", "Rajiv Chowk~BY", 7, 15, 47},
        {"Rajiv Chowk~BY", "New Delhi~YO", 1, 5, 20},
        {"New Delhi~YO", "Chandni Chowk~Y", 2, 7, 25},
        {"Chandni Chowk~Y", "Vishwavidyalaya~Y", 5, 10, 35},
        {"New Delhi~YO", "Shivaji Stadium~O", 2, 6, 22},
        {"Shivaji Stadium~O", "DDS Campus~O", 7, 17, 55},
        {"DDS Campus~O", "IGI Airport~O", 8, 20, 65},
        {"Moti Nagar~B", "Rajouri Garden~BP", 2, 8, 30},
        {"Punjabi Bagh West~P", "Rajouri Garden~BP", 2, 8, 30},
        {"Punjabi Bagh West~P", "Netaji Subhash Place~PR", 3, 9, 32}
    };

    for (auto& edge : edges) {
        metro_graph.addEdge(get<0>(edge), get<1>(edge), get<2>(edge), get<3>(edge), get<4>(edge));
    }

    metro_graph.userInterface();

    return 0;
}