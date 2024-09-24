#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <memory>
#include <algorithm>

template<typename Vertex, typename Distance = double>//vertex - versh grapha distance - rasstoianie edge - rebro
class Graph {
public:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance distance;

        Edge(const Vertex& f, const Vertex& t, const Distance& d)
            : from(f), to(t), distance(d) {}
    };

    bool has_vertex(const Vertex& v) const {
        return adjacency_list.find(v) != adjacency_list.end();
    }
    //???? ????
    void add_vertex(const Vertex& v) {
        adjacency_list[v];
    }

    bool remove_vertex(const Vertex& v) {
        if (!has_vertex(v)) return false;

        adjacency_list.erase(v);
        for (auto& [key, edges] : adjacency_list) {
            edges.erase(
                std::remove_if(edges.begin(), edges.end(), [&v](const Edge& e) {
                    return e.to == v;
                    }),
                edges.end()
            );
        }
        return true;
    }
    //???? ?????? ???? ???? ? ?????vozvrat spisok
    std::vector<Vertex> vertices() const {
        std::vector<Vertex> verts;
        for (const auto& pair : adjacency_list) {
            verts.push_back(pair.first);//firtst-vershina second-reber
        }
        return verts;
    }
    // ??? ???
    void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
        adjacency_list[from].emplace_back(from, to, d);
    }
    // ???? ? ????????? ?? ??? 
    bool remove_edge(const Vertex& from, const Vertex& to) {
        if (!has_vertex(from)) return false;
        auto& edges = adjacency_list[from];
        auto it = std::remove_if(edges.begin(), edges.end(), [&to](const Edge& e) {
            return e.to == to;
            });
        if (it != edges.end()) {
            edges.erase(it, edges.end());
            return true;
        }
        return false;
    }

    bool remove_edge(const Edge& e) {
        if (!has_vertex(e.from)) return false;
        auto& edges = adjacency_list[e.from];
        auto it = std::remove_if(edges.begin(), edges.end(), [&e](const Edge& edge) {
            return edge.from == e.from && edge.to == e.to && edge.distance == e.distance;
            });
        if (it != edges.end()) {
            edges.erase(it, edges.end());
            return true;
        }
        return false;
    }

    bool has_edge(const Vertex& from, const Vertex& to) const {
        if (!has_vertex(from)) return false;
        const auto& edges = adjacency_list.at(from);
        return std::any_of(edges.begin(), edges.end(), [&to](const Edge& e) {
            return e.to == to;
            });
    }

    bool has_edge(const Edge& e) const {
        if (!has_vertex(e.from)) return false;
        const auto& edges = adjacency_list.at(e.from);
        return std::any_of(edges.begin(), edges.end(), [&e](const Edge& edge) {
            return edge.from == e.from && edge.to == e.to && edge.distance == e.distance;
            });
    }
    //?????.?????? ?? ??? ????? ?? ???? ???????
    std::vector<Edge> edges(const Vertex& vertex) const {//vozvrat full rebra
        if (!has_vertex(vertex)) return {};
        return adjacency_list.at(vertex);
    }

    size_t order() const {//????? vozvrat kolvo ver
        return adjacency_list.size();
    }

    size_t degree(const Vertex& v) const {//???????vozvrat stepeni vershin
        if (!has_vertex(v)) return 0;
        return adjacency_list.at(v).size();
    }

    std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        std::unordered_map<Vertex, Distance> distances;//????? ?????
        std::unordered_map<Vertex, Vertex> previous;//???? ?????? ? ????
        auto compare = [&distances](const Vertex& a, const Vertex& b) {//
            return distances[a] > distances[b];
            };
        std::priority_queue<Vertex, std::vector<Vertex>, decltype(compare)> queue(compare);

        for (const auto& pair : adjacency_list) {
            distances[pair.first] = std::numeric_limits<Distance>::infinity();
        }
        distances[from] = 0;
        queue.push(from);

        while (!queue.empty()) {
            Vertex current = queue.top();
            queue.pop();

            if (current == to) break;

            for (const Edge& edge : adjacency_list.at(current)) {
                Vertex neighbor = edge.to;
                Distance new_dist = distances[current] + edge.distance;
                if (new_dist < distances[neighbor]) {
                    distances[neighbor] = new_dist;
                    previous[neighbor] = current;
                    queue.push(neighbor);
                }
            }
        }

        std::vector<Edge> path;
        for (Vertex at = to; previous.find(at) != previous.end(); at = previous[at]) {
            Vertex from = previous[at];
            Distance dist = distances[at] - distances[from];
            path.push_back(Edge(from, at, dist));
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    std::vector<Vertex> walk(const Vertex& start_vertex) const {
        std::vector<Vertex> result;
        if (!has_vertex(start_vertex)) return result;

        std::unordered_set<Vertex> visited;
        std::queue<Vertex> queue;
        queue.push(start_vertex);
        visited.insert(start_vertex);

        while (!queue.empty()) {
            Vertex current = queue.front();
            queue.pop();
            result.push_back(current);

            for (const Edge& edge : adjacency_list.at(current)) {
                if (visited.find(edge.to) == visited.end()) {
                    queue.push(edge.to);
                    visited.insert(edge.to);
                }
            }
        }
        return result;
    }

    Distance max_distance_from(const Vertex& start_vertex) const {
        std::unordered_map<Vertex, Distance> distances;
        for (const auto& pair : adjacency_list) {//asociativ conteiner
            distances[pair.first] = std::numeric_limits<Distance>::infinity();
        }
        distances[start_vertex] = 0;//start 

        std::priority_queue<std::pair<Distance, Vertex>, std::vector<std::pair<Distance, Vertex>>, std::greater<>> queue;
        queue.emplace(0, start_vertex);

        while (!queue.empty()) {
            Distance current_distance = queue.top().first;
            Vertex current_vertex = queue.top().second;
            queue.pop();

            if (current_distance > distances[current_vertex]) {
                continue;
            }

            for (const Edge& edge : adjacency_list.at(current_vertex)) {
                Distance new_distance = current_distance + edge.distance;
                if (new_distance < distances[edge.to]) {
                    distances[edge.to] = new_distance;
                    queue.emplace(new_distance, edge.to);
                }
            }
        }

        Distance max_distance = 0;//rastoianie
        for (const auto& pair : distances) {
            if (pair.second != std::numeric_limits<Distance>::infinity() && pair.second > max_distance) {
                max_distance = pair.second;
            }
        }
        return max_distance;
    }

    Vertex optimal_storage_point() const {
        Distance min_max_distance = std::numeric_limits<Distance>::infinity();
        Vertex optimal_vertex;
        for (const auto& pair : adjacency_list) {
            Distance max_distance = max_distance_from(pair.first);
            if (max_distance < min_max_distance) {
                min_max_distance = max_distance;
                optimal_vertex = pair.first;
            }
        }
        return optimal_vertex;
    }

private:
    std::unordered_map<Vertex, std::vector<Edge>> adjacency_list;
};

// ???????????? ?????? ?????? Graph ? ??????? ??????
int main() {
    Graph<std::string> graph;


    graph.add_vertex("A");
    graph.add_vertex("B");
    graph.add_vertex("C");
    graph.add_vertex("D");

    //  ?????
    graph.add_edge("A", "B", 1.0);
    graph.add_edge("B", "C", 2.0);
    graph.add_edge("C", "D", 3.0);
    graph.add_edge("A", "D", 4.0);

    // ????? ??????
    std::cout << "Vertices: ";
    for (const auto& vertex : graph.vertices()) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;

    // ????? ?????
    std::cout << "Edges from A: ";
    for (const auto& edge : graph.edges("A")) {
        std::cout << "(" << edge.from << " -> " << edge.to << ", " << edge.distance << ") ";
    }
    std::cout << std::endl;

    // ????? ??????????? ????
    auto path = graph.shortest_path("A", "D");
    std::cout << "Shortest path from A to D: ";
    for (const auto& edge : path) {
        std::cout << "(" << edge.from << " -> " << edge.to << ", " << edge.distance << ") ";
    }
    std::cout << std::endl;

    // ?????
    std::string optimal_point = graph.optimal_storage_point();
    std::cout << "Optimal storage point: " << optimal_point << std::endl;

    return 0;
}