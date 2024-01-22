/*
 * A DdSpec for Capacitated Arc Routing Problem
 * Checks whether a given edge set is serviced
 * Limits each graph copy to a set capacity
 */

#pragma once

#include <tdzdd/DdSpec.hpp>
#include "Graph.hpp"

namespace tdzdd {

class TourSpec: public PodArrayDdSpec<TourSpec,uint16_t,2> {
    typedef uint16_t Mate;

    Graph const& graph;
    std::vector<int> const requiredEdges;
    int const numRequiredEdges;
    int const numEdgesGraph;
    int const hashCellSize;
    int const capacity;
    int const hashSize;
    int const copies;
    int const m;
    int const n;

    int getServiced(int v, Mate const* mate) const {
        assert (0 <= v && v < n);
        int p = std::distance(requiredEdges.begin(), std::find(requiredEdges.begin(), requiredEdges.end(), v % numEdgesGraph));
        assert (0 <= p && p < numRequiredEdges);
        return (mate[p / hashCellSize] >> (p % hashCellSize)) & 1u;
    }

    void setServiced(int v, Mate* mate) const {
        assert (0 <= v && v < n);
        int p = std::distance(requiredEdges.begin(), std::find(requiredEdges.begin(), requiredEdges.end(), v % numEdgesGraph));
        assert (0 <= p && p < numRequiredEdges);
        mate[p / hashCellSize] |= (1u << (p % hashCellSize));
    }

    int getUnserviced(Mate const* mate) const {
        int unserviced = 0;
        for (int v : requiredEdges) {
            if (!getServiced(v, mate)) unserviced += graph.getEdgeDemand(v);
        }
        return unserviced;
    }

    int getFirst(Mate const* mate) const {
        int p = numRequiredEdges;
        return (mate[p / hashCellSize] >> (p % hashCellSize)) & 1u;
    }

    void setFirst(int first, Mate* mate) const {
        int p = numRequiredEdges;
        if (first) {
            mate[p / hashCellSize] |= (1u << (p % hashCellSize));
        }
        else {
            mate[p / hashCellSize] &= ~(1u << (p % hashCellSize));
        }
    }

    bool doService(Mate* mate, int i) const {
        // Trivial cases
        if (i % 3 == 0 && graph.getEdgeDemand(i) == 0) return false;
        if (i % 3 > 0) return true;
        
        if (getServiced(i, mate)) return false;
        if (mate[hashSize] < graph.getEdgeDemand(i)) return false;

        mate[hashSize] -= graph.getEdgeDemand(i);
        setServiced(i, mate);
        setFirst(0, mate);

        return true;
    }

    bool doNotService(Mate* mate, int i) const {
        // Must service first nonserviced edge if first service of graph copy
        if (getFirst(mate)) {
            for (int v : requiredEdges) {
                if (!getServiced(v, mate)) {
                    if (v == i % numEdgesGraph) return false;
                    break;
                }
            }
        }

        // Last graph copy and not yet serviced
        if (i / numEdgesGraph == copies - 1) {
            if (std::count(requiredEdges.begin(), requiredEdges.end(), i % numEdgesGraph)) {
                if (!getServiced(i, mate)) return false;
            }
        }

        return true;
    }
    
public:
    TourSpec(Graph const& graph, std::vector<int> const& requiredEdges, int capacity)
            : graph(graph), requiredEdges(requiredEdges),
              capacity(capacity), copies(graph.numCopyGraph()),
              numRequiredEdges(requiredEdges.size()),
              numEdgesGraph(graph.numEdgesGraph()),
              m(graph.vertexSize()), n(graph.edgeSize()),
              hashCellSize(sizeof(Mate) * 8),
              hashSize(numRequiredEdges / hashCellSize + 1) {
        this->setArraySize(hashSize + 1);
    }

    int getRoot(Mate* mate) const {
        for (int i = 0; i < hashSize; ++i) mate[i] = 0;
        setFirst(1, mate);
        mate[hashSize] = capacity;

        return n;
    }

    int getChild(Mate* mate, int level, int take) const {
        assert(1 <= level && level <= n);
        int i = n - level;

        if (take) {
            if (!doService(mate, i)) return 0;
            if (i % 3 == 0) ++i;
        }
        else {
            if (i % 3 == 1) ++i;
            if (!doNotService(mate, i)) return 0;
        }

        // Lookahead on remaining capacity
        if ((copies - i / numEdgesGraph - 1) * capacity + mate[hashSize] < getUnserviced(mate)) return 0;

        if (graph.edgeInfo(i).finalEdge) {
            setFirst(1, mate);
            mate[hashSize] = capacity;
        }

        if (++i == n) return -1;

        assert(i < n);
        return n - i;
    }
};

} // namespace tdzdd