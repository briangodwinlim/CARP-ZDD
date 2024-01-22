#include <map>
#include <queue>
#include <functional>

// TdZdd
#include <tdzdd/DdEval.hpp>
#include <tdzdd/DdSpecOp.hpp>
#include <tdzdd/DdStructure.hpp>
#include <tdzdd/util/IntSubset.hpp>

#include "Graph.hpp"
#include "TourSpec.hpp"
#include "DegreeConstraint.hpp"
#include "FrontierBasedSearch.hpp"

using namespace tdzdd;

typedef std::pair<int,std::vector<int>> OptSolution;


// Find the minimum cost
class GetSolution: public DdEval<GetSolution,OptSolution> {
private:
    Graph const& graph;
    int const n;

public:
    GetSolution(Graph const& graph) 
        : graph(graph), n(graph.edgeSize()) {
    }

    void evalTerminal(OptSolution &v, int id) {
        v = id ? std::make_pair(0, std::vector<int>()) : std::make_pair(INT_MAX / 2, std::vector<int>());
    }

    void evalNode(OptSolution &v, int level, DdValues<OptSolution,2> const& values) {
        int i = n - level;
        if (values.get(0).first > values.get(1).first + graph.getEdgeCost(i)) {
            v = values.get(1);
            v.first += graph.getEdgeCost(i);
            v.second.push_back(level);
        }
        else {
            v = values.get(0);
        }
    }
};


std::string options[][2] = { //
        {"graph", "Dump duplicated graph to STDOUT in DOT format"}, //
        {"vehicles <n>", "Override number of vehicles available"}, //
        {"traversal <n>", "Edge traversal order: 0 - as is, 1 - bfs, 2 - dfs"}, //
        {"allvehicles", "All vehicles must have tours"}, //
        {"openMP", "Use openMP in the construction of the ZDD"}, //          export OMP_NUM_THREADS=THREADS
        {"zdd", "Dump resulting ZDD to STDOUT in DOT format"}, //
        {"export", "Dump resulting ZDD to STDOUT"}, //
        {"solution", "Output an optimal solution"} 
};

std::map<std::string,bool> opt;
std::map<std::string,int> optNum;

void usage(char const* cmd) {
    std::cerr << "usage: " << cmd
              << " <graph_file> [ <option>... ]\n";
    std::cerr << "options\n";
    for (unsigned i = 0; i < sizeof(options) / sizeof(options[0]); ++i) {
        std::cerr << "  -" << options[i][0];
        for (unsigned j = options[i][0].length(); j < 15; ++j) {
            std::cerr << " ";
        }
        std::cerr << ": " << options[i][1] << "\n";
    }
}

int main(int argc, char *argv[]) {
    for (unsigned i = 0; i < sizeof(options) / sizeof(options[0]); ++i) {
        opt[options[i][0]] = false;
    }

    std::string graphFileName;

    try {
        for (int i = 1; i < argc; ++i) {
            std::string s = argv[i];
            if (s[0] == '-') {
                s = s.substr(1);

                if (opt.count(s)) {
                    opt[s] = true;
                }
                else if (i + 1 < argc && opt.count(s + " <n>")) {
                    opt[s] = true;
                    optNum[s] = std::stoi(argv[++i]);
                }
                else {
                    throw std::exception();
                }
            }
            else if (graphFileName.empty()) {
                graphFileName = s;
            }
            else {
                throw std::exception();
            }
        }
    }
    catch (std::exception& e) {
        usage(argv[0]);
        return 1;
    }

    MessageHandler::showMessages();
    MessageHandler mh;
    mh.begin("started");

    // Information required
    Graph g;
    int numVertices, numEdgesR, numEdgesNR, numEdges, numVehicles, capacity, ans;
    std::vector<Graph::EdgeNumber> requiredEdges;
    std::string depot;
    
    try { 
        // Read input file
        std::string line, tmp;
        std::ifstream file;
        std::istringstream ss;
        file.open(std::string(argv[1]));

        // Edge traversal trackers
        std::queue<std::string> q;
        std::set<std::string> visited_v;
        std::set<std::pair<std::string,std::string>> visited_e;
        std::vector<std::pair<std::string,std::string>> edgeList;
        std::map<std::string,std::vector<std::string>> adjacencyList;
        std::map<std::pair<std::string,std::string>,int> costMap, demandMap;

        getline(file, line); // Name
        getline(file, line); // Comment

        getline(file, line); // Vertices
        ss = std::istringstream(line);
        ss >> tmp >> tmp >> tmp;
        numVertices = std::stoi(tmp);
        
        getline(file, line); // Edges required
        ss = std::istringstream(line);
        ss >> tmp >> tmp >> tmp;
        numEdgesR = std::stoi(tmp);
        
        getline(file, line); // Edges nonrequired
        ss = std::istringstream(line);
        ss >> tmp >> tmp >> tmp;
        numEdgesNR = std::stoi(tmp);
        numEdges = numEdgesR + numEdgesNR;
        
        getline(file, line); // Vehicles
        ss = std::istringstream(line);
        ss >> tmp >> tmp >> tmp;
        numVehicles = std::stoi(tmp);
        if (opt["vehicles"]) numVehicles = optNum["vehicles"];
        
        getline(file, line); // Capacity
        ss = std::istringstream(line);
        ss >> tmp >> tmp >> tmp;
        capacity = std::stoi(tmp);
        
        getline(file, line); // Explicit declaration
        getline(file, line); // Given answer
        ss = std::istringstream(line);
        ss >> tmp >> tmp >> tmp;
        ans = std::stoi(tmp);
        
        getline(file, line); // Required edges header

        // Read required edges
        for (int e = 0; e < numEdgesR; ++e) {
            getline(file, line);
            ss = std::istringstream(line);
            
            std::string u, v, c, d;
            ss >> tmp >> u >> v >> tmp >> c >> tmp >> d;
            u = u.substr(0, u.length()-1);
            v = v.substr(0, v.length()-1);
            
            edgeList.push_back(std::make_pair(u,v));
            if (adjacencyList.find(u) == adjacencyList.end()) adjacencyList[u] = std::vector<std::string>();
            if (adjacencyList.find(v) == adjacencyList.end()) adjacencyList[v] = std::vector<std::string>();
            adjacencyList[u].push_back(v);
            adjacencyList[v].push_back(u);
            costMap[std::make_pair(u,v)] = std::stoi(c);
            costMap[std::make_pair(v,u)] = std::stoi(c);
            demandMap[std::make_pair(u,v)] = std::stoi(d);
            demandMap[std::make_pair(v,u)] = std::stoi(d);
        }

        if (numEdgesNR) getline(file, line); // Nonrequired edges header

        // Read nonrequired edges
        for (int e = 0; e < numEdgesNR; ++e) {
            getline(file, line);
            ss = std::istringstream(line);
            
            std::string u, v, c;
            ss >> tmp >> u >> v >> tmp >> c;
            u = u.substr(0, u.length()-1);
            v = v.substr(0, v.length()-1);

            edgeList.push_back(std::make_pair(u,v));
            if (adjacencyList.find(u) == adjacencyList.end()) adjacencyList[u] = std::vector<std::string>();
            if (adjacencyList.find(v) == adjacencyList.end()) adjacencyList[v] = std::vector<std::string>();
            adjacencyList[u].push_back(v);
            adjacencyList[v].push_back(u);
            costMap[std::make_pair(u,v)] = std::stoi(c);
            costMap[std::make_pair(v,u)] = std::stoi(c);
            demandMap[std::make_pair(u,v)] = 0;
            demandMap[std::make_pair(v,u)] = 0;
        }
        
        getline(file, line); // Depot
        ss = std::istringstream(line);
        ss >> tmp >> tmp >> tmp;
        depot = tmp;
        
        file.close();

        // Sort edges as is
        std::function<void()> as_is = [&]() {
            for (std::pair<std::string,std::string> e : edgeList) {
                std::string u = e.first;
                std::string v = e.second;
                g.addEdge(u, v, costMap[std::make_pair(u,v)], demandMap[std::make_pair(u,v)]);    // Service edge
                g.addEdge(u, v, costMap[std::make_pair(u,v)], demandMap[std::make_pair(u,v)]);    // Nonservice edge
                g.addEdge(u, v, costMap[std::make_pair(u,v)], demandMap[std::make_pair(u,v)]);    // Nonservice edge
            }
        };

        // Sort edges by bfs
        std::function<void(std::string)> bfs = [&](std::string u) {
            q.push(u);
            visited_v.insert(u);
            while (!q.empty()) {
                u = q.front();
                q.pop();
                for (std::string v : adjacencyList[u]) {
                    if (visited_v.count(v) == 0) {
                        q.push(v);
                        visited_v.insert(v);
                    }
                    if (visited_e.count(std::make_pair(u,v)) == 0) {
                        g.addEdge(u, v, costMap[std::make_pair(u,v)], demandMap[std::make_pair(u,v)]);    // Service edge
                        g.addEdge(u, v, costMap[std::make_pair(u,v)], demandMap[std::make_pair(u,v)]);    // Nonservice edge
                        g.addEdge(u, v, costMap[std::make_pair(u,v)], demandMap[std::make_pair(u,v)]);    // Nonservice edge
                        visited_e.insert(std::make_pair(u,v));
                        visited_e.insert(std::make_pair(v,u));
                    }
                }
            }
        };

        // Sort edges by dfs
        std::function<void(std::string)> dfs = [&](std::string u) {
            for (std::string v : adjacencyList[u]) {
                if (visited_e.count(std::make_pair(u,v)) == 0) {
                    g.addEdge(u, v, costMap[std::make_pair(u,v)], demandMap[std::make_pair(u,v)]);    // Service edge
                    g.addEdge(u, v, costMap[std::make_pair(u,v)], demandMap[std::make_pair(u,v)]);    // Nonservice edge
                    g.addEdge(u, v, costMap[std::make_pair(u,v)], demandMap[std::make_pair(u,v)]);    // Nonservice edge
                    visited_e.insert(std::make_pair(u,v));
                    visited_e.insert(std::make_pair(v,u));
                    dfs(v);
                }
            }
        };

        // Add edges to g
        if (opt["traversal"]) {
            if (optNum["traversal"] == 0) as_is();
            else if (optNum["traversal"] == 1) bfs(depot);
            else if (optNum["traversal"] == 2) dfs(depot);
            else throw std::invalid_argument("Invalid traversal");
        }
        else {
            as_is();
        }
    }
    catch (std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    try {
        // Duplicate graph
        g.duplicateGraph(numVehicles);

        // Output graph information
        int const m = g.vertexSize();
        int const n = g.edgeSize();
        mh << "\n#vertex = " << m << ", #edge = " << n << ",\n";
        mh << "#frontierSize = " << g.maxFrontierSize() << ", #copies = " << g.numCopyGraph() << "\n";
        assert(n == 3 * numEdges * g.numCopyGraph());

        // Check graph size
        if (n == 0) 
            throw std::runtime_error("ERROR: The graph is empty!");

        // Output graph as dot format
        if (opt["graph"]) g.dump(std::cout);

        // Check required edges
        for (int e = 0; e < n / g.numCopyGraph(); e += 3) {
            if (g.getEdgeDemand(e)) requiredEdges.push_back(e);
        }
        assert(requiredEdges.size() == numEdgesR);
        assert(3 * numEdgesR * g.numCopyGraph() == g.numRequiredEdges());

        // Run TourSpec
        TourSpec tspec(g, requiredEdges, capacity);
        DdStructure<2> dd(tspec, opt["openMP"]);
        dd.zddReduce();

        // Run Frontier Based Search
        FrontierBasedSearch fbs(g, 1, !opt["allvehicles"]);
        DdStructure<2> fbs_dd(fbs);
        fbs_dd.zddReduce();
        dd.zddSubset(fbs_dd);
        dd.zddReduce();

        // Run Degree Constraints
        IntRange Even(0, n, 2);
        IntRange Depot(2, n, 2);
        DegreeConstraint dc(g, &Even, !opt["allvehicles"]);
        for (int i = 0; i < numVehicles; ++i) 
            dc.setConstraint(depot + "_" + to_string(i), &Depot);
        DdStructure<2> dc_dd(dc);
        dc_dd.zddReduce();
        dd.zddSubset(dc_dd);
        dd.zddReduce();

        // Output resulting ZDD
        if (opt["zdd"]) dd.dumpDot(std::cout, "ZDD");
        if (opt["export"]) dd.dumpSapporo(std::cout);

        std::string numSolutions = dd.evaluate(ZddCardinality<>());
        if (numSolutions != "0") {
            // Output ZDD information
            std::pair<int,std::vector<int>> optSolution = dd.evaluate(GetSolution(g));
            int minCost = optSolution.first;
            std::vector<int> edgeList = optSolution.second;
            mh << "\n#node = " << dd.size() 
                << ", #solution = " << numSolutions
                << ", MinimumCost = " << minCost
                << "\n";
            
            // Output solution
            if (opt["solution"]) {            
                for (int e : edgeList) {
                    if ((n - e) % 3 == 0) std::cout << "Service : " << g.edgeLabel(n - e);
                    else std::cout << "Traverse: " << g.edgeLabel(n - e);
                    std::cout << "\n";
                }
            }
        }
        else {
            mh << "\nNo solutions found\n";
        }
    }
    catch (std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    mh.end("finished");
    return 0;
}
