// Simple dynamic program to compute shortest distance over a graph
//
// - Read data from a OpenStreetMap Dat file
// - Represent streets as a graph and distances between nodes in meters
// - Calculate shortest walkable path between two OpenStreetMap IDs
//
// Compile with
//   g++ shortestpath.cpp -o shortestpath -std=c++17
// Run as
//   ./shortestpath <path to graph> <from-osm-id> <to-osm-id>

#include <algorithm>      // heap data-structure
#include <fstream>        // reading from files
#include <iostream>       // cout/cerr
#include <memory>         // unique_ptr type-erasure to make DatFile testable
#include <random>         // for testing
#include <sstream>        // formatting
#include <unordered_map>  // hash table for osm id -> vector index
#include <vector>         // arrays

using Distance = uint32_t;  // distance in meters
using OsmId = uint64_t;     // open street map ids
using Index = int;          // indices in vectors

// graph nodes
struct Node final {
    struct Edge {
        Index target;
        Distance dist;
    };
    std::vector<Edge> edges;

    // shortest distance from from-node to this node (computed incrementally)
    Distance min_distance = std::numeric_limits<Distance>::max();

    // mark nodes which need to be updated
    bool need_propagate = false;
};

// quick&dirty parser for OSM dat files
class DatFile final {
public:
    DatFile() = delete;
    explicit DatFile(const std::string& filename) : m_filename(filename) {
        std::ifstream file;
        // turn on exceptions first, by default streams silently ignore errors
        file.exceptions(std::istream::failbit | std::istream::badbit);
        file.open(filename);
        m_file = std::make_unique<std::ifstream>(std::move(file));
    }

    // for testing
    explicit DatFile(std::stringstream&& file)
        : m_filename("<input>"),
          m_file(std::make_unique<std::stringstream>(std::move(file))) {
        m_file->exceptions(std::istream::failbit | std::istream::badbit);
    }

    template <typename... Values>
    void parse_line(Values&... values) {
        m_linenum++;

        // read a line
        std::string tmp;
        std::getline(*m_file, tmp, '\n');

        // parse a sequence of values of the given types
        std::stringstream line(std::move(tmp));
        line.exceptions(std::istream::failbit | std::istream::badbit);
        (line >> ... >> values);
    }

    std::string position() const {
        return m_filename + ":" + std::to_string(m_linenum);
    }

private:
    std::string m_filename;
    std::unique_ptr<std::istream> m_file;
    int m_linenum = 0;
};

// Problem definition:
// given graph and to-from nodes
// we want the single-pair shortest distance
struct ShortestDistanceProblem {
    std::vector<Node> graph;
    Index from_index;
    Index to_index;
};

ShortestDistanceProblem construct_problem(DatFile input,
                                          OsmId from_id,
                                          OsmId to_id) {
    // Dat file format:
    //   <number of nodes>
    //   <OSM id of node>
    //   ...
    //   <OSM id of node>
    //   <number of edges>
    //   <from node OSM id> <to node OSM id> <length in meters>
    //   ...
    //   <from node OSM id> <to node OSM id> <length in meters>

    try {
        // read OSM id of nodes
        // nodes are stored in a vector that represents the graph
        // construct a map osm_id -> index in vector
        Index num_nodes;
        input.parse_line(num_nodes);
        std::vector<Node> graph(num_nodes);
        std::unordered_map<OsmId, Index> osm_to_index(num_nodes);
        for (Index i = 0; i != num_nodes; ++i) {
            OsmId node_id;
            input.parse_line(node_id);
            if (!osm_to_index.emplace(node_id, i).second) {
                throw std::runtime_error("Duplicate OSM id: " +
                                         std::to_string(node_id));
            }
        }

        auto find_node = [&](OsmId node_id) {
            auto it = osm_to_index.find(node_id);
            if (it == osm_to_index.end()) {
                throw std::runtime_error("OSM id not found: " +
                                         std::to_string(node_id));
            }
            return it->second;
        };

        // find from and to nodes
        Index from_index = find_node(from_id);
        Index to_index = find_node(to_id);

        // read edges from file
        // edges are undirected, and stored explicitly in both nodes
        Index num_edges;
        input.parse_line(num_edges);
        for (Index i = 0; i != num_edges; ++i) {
            OsmId a, b;
            Distance dist;
            input.parse_line(a, b, dist);

            Index node_a = find_node(a);
            Index node_b = find_node(b);
            if (node_a == node_b) {
                throw std::runtime_error("Invalid loop edge: " +
                                         std::to_string(a));
            }
            graph[node_a].edges.push_back({node_b, dist});
            graph[node_b].edges.push_back({node_a, dist});
        }

        return {std::move(graph), from_index, to_index};
    } catch (std::exception& e) {
        throw std::runtime_error(input.position() + ": " + e.what());
    }
}

// Problem solution:
// Even though we're only interested in single-pair distance, we will
// incrementally compute the shortest distance from "from-node" to all others,
// so that we can reuse partial distances  in the style of dynamic programming.
// When we update the distance to a node, we put it in a queue, so that we can
// later propagate the newly found shorter distance to it, further over its
// edges. The queue is actually a heap, which gives log(n) access to the closest
// node which needs updating, and this helps us stop earlier, when we know all
// remaining nodes which need updating will not improve the solution.
Distance solve_problem(ShortestDistanceProblem&& p) {
    // std::make_heap is a max-heap
    // so we want lhs < rhs if lhs.distance > rhs.distance
    auto comp = [&](Index lhs, Index rhs) {
        return p.graph[lhs].min_distance > p.graph[rhs].min_distance;
    };

    // put from-node in the queue
    p.graph[p.from_index].min_distance = 0;
    std::vector<Index> closest_nodes;
    closest_nodes.push_back(p.from_index);

    Distance& shortest_distance_so_far = p.graph[p.to_index].min_distance;
    while (!closest_nodes.empty()) {
        // pop from queue
        std::pop_heap(closest_nodes.begin(), closest_nodes.end(), comp);
        Node& node = p.graph[closest_nodes.back()];
        closest_nodes.pop_back();
        node.need_propagate = false;
        if (node.min_distance >= shortest_distance_so_far) {
            // optimization: we cannot possibly compute a shorter path
            // by moving through a node that is already farther than
            // the shortest we found so far
            break;
        }

        // iterate over edges
        bool modified = false;
        for (const auto& edge : node.edges) {
            Distance& target_dist = p.graph[edge.target].min_distance;
            // if walking across this edge leads to a shorter path
            // than previously known
            // (from source from-node to this node)
            if (target_dist > node.min_distance + edge.dist) {
                // update min dist
                target_dist = node.min_distance + edge.dist;
                bool& target_needs_propagate =
                    p.graph[edge.target].need_propagate;
                modified = true;

                // propagate path through this node, stopping at the target
                // the needs_propagate mark avoids pushing same node twice
                if (edge.target != p.to_index && !target_needs_propagate) {
                    target_needs_propagate = true;
                    closest_nodes.push_back(edge.target);
                }
            }
        }

        // reinstate heap
        if (modified) {
            std::make_heap(closest_nodes.begin(), closest_nodes.end(), comp);
        }
    }

    if (shortest_distance_so_far == std::numeric_limits<Distance>::max()) {
        throw std::runtime_error("Path between nodes not found");
    }

    return shortest_distance_so_far;
}

static void unittest() {
    // ideally there'd be tests here for construct_problem too
    // stress test solve_problem
    int repeats = 100;  // number of iterations, each solves n*n problems
    int n = 80;         // number of nodes
    int bias = 10;
    std::default_random_engine gen;
    std::uniform_int_distribution<int> dist(0, n - 1);
    for (int iter = 0; iter != repeats; ++iter) {
        // probability that there's an edge between two nodes
        // for example if bias = 10 repeats = 80 then
        // density ranges from 10/100 = 10% .. 90/100 = 90% in steps of 1%
        float density = float(iter + bias) / (repeats + 2 * bias);
        std::vector<Node> graph(n);
        std::vector<Distance> reference(n * n,
                                        std::numeric_limits<Distance>::max());
        for (int i = 0; i != n; ++i) {
            for (int j = 0; j != i + 1; ++j) {
                if (i == j) {  // diagonal
                    reference[i * n + j] = 0;
                    reference[j * n + i] = 0;
                } else if (i != j && dist(gen) < density * n) {
                    Distance distance = dist(gen) + 10;
                    graph[i].edges.push_back({j, distance});
                    graph[j].edges.push_back({i, distance});
                    reference[i * n + j] = distance;
                    reference[j * n + i] = distance;
                }
            }
        }

        // for debugging, print matrix
        //  for (int i = 0; i != n; ++i) {
        //      for (int j = 0; j != n; ++j) {
        //          std::cout << std::setw(12) << reference[i*n + j] << " ";
        //      }
        //      std::cout << "\n" << std::endl;
        //  }

        // generate reference solution
        // naive floyd-warshall all-pairs shortest-distance
        for (int k = 0; k != n; ++k) {
            for (int i = 0; i != n; ++i) {
                Distance v = reference[i * n + k];
                if (v == std::numeric_limits<Distance>::max()) {
                    continue;
                }
                for (int j = 0; j != n; ++j) {
                    if (reference[k * n + j] ==
                        std::numeric_limits<Distance>::max()) {
                        continue;
                    }
                    Distance val = v + reference[k * n + j];
                    if (reference[i * n + j] > val) {
                        reference[i * n + j] = val;
                    }
                }
            }
        }

        // solve problems and compare with reference solution
        for (int i = 0; i != n; ++i) {
            for (int j = 0; j != n; ++j) {
                Distance solution = std::numeric_limits<Distance>::max();
                try {
                    ShortestDistanceProblem prob;
                    prob.graph = graph;
                    prob.from_index = i;
                    prob.to_index = j;
                    solution = solve_problem(std::move(prob));
                } catch (std::exception&) {
                }
                if (solution != reference[i * n + j]) {
                    throw std::runtime_error(
                        "Mismatch i=" + std::to_string(i) + " j=" +
                        std::to_string(j) + " sol=" + std::to_string(solution) +
                        " ref=" + std::to_string(reference[i * n + j]));
                }
            }
        }
    }
}

int main(int argc, char* argv[]) {
    try {
        // uncomment to enable testing
        // unittest();

        if (argc != 4) {
            std::cerr << "Usage: <osm.dat> <from-node-osm-id> <to-node-osm-id>"
                      << std::endl;
            return 2;
        }

        // handle command line options
        DatFile input(argv[1]);
        OsmId from_id = std::stol(argv[2]);
        OsmId to_id = std::stol(argv[3]);

        // construct and solve the problem
        std::cout << solve_problem(
                         construct_problem(std::move(input), from_id, to_id))
                  << std::endl;
    } catch (std::exception& e) {
        // slightly less ugly error message than std::terminate
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
