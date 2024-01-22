/* 
 * A modified version of tdzdd/spec/DegreeConstraint.hpp for Capacitated Arc Routing Problem
 * Added nullPath to include null paths (no tours)
 */

#pragma once

#include <cassert>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>

#include <tdzdd/DdSpec.hpp>
#include <tdzdd/util/IntSubset.hpp>
#include "Graph.hpp"

namespace tdzdd {

class DegreeConstraint: public PodArrayDdSpec<DegreeConstraint,int16_t,2> {
    typedef int16_t Mate;

    Graph const& graph;
    std::vector<IntSubset const*> constraints;
    int const n;
    int const mateSize;
    bool const nullPath;
    bool const lookahead;

    void shiftMate(Mate* mate, int d) const {
        assert(d >= 0);
        if (d > 0) {
            std::memmove(mate, mate + d, (mateSize - d) * sizeof(*mate));
            for (int k = mateSize - d; k < mateSize; ++k) {
                mate[k] = 0;
            }
        }
    }

    bool takable(IntSubset const* c, Mate degree, bool final, Mate const& tour) const {
        if (tour == 2) return false;    // CARP: If null tour, cannot take
        if (c == 0) return true;
        if (degree >= c->upperBound()) return false;
        return !final || c->contains(degree + 1);
    }

    bool leavable(IntSubset const* c, Mate degree, bool final, Mate& tour) const {
        if (c == 0) return true;
        // CARP: If final, no degree, invalid degree constraint, and not standard tour, set to null tour
        if (final && degree == 0 && !c->contains(degree) && tour != 1 && nullPath) {
            tour = 2;
            return true;
        }
        return !final || c->contains(degree);
    }

public:
    DegreeConstraint(Graph const& graph, IntSubset const* c = 0, bool nullPath = false, bool lookahead = true)
            : graph(graph), n(graph.edgeSize()),
              mateSize(graph.maxFrontierSize()), 
              nullPath(nullPath), lookahead(lookahead) {
        setArraySize(mateSize + 1);

        int m = graph.vertexSize();
        constraints.resize(m + 1);
        for (int v = 1; v <= m; ++v) {
            constraints[v] = c;
        }
    }

    void setConstraint(Graph::VertexNumber v, IntSubset const* c) {
        if (v < 1 || graph.vertexSize() < v) throw std::runtime_error(
                "ERROR: Vertex number is out of range");
        constraints[v] = c;
    }

    void setConstraint(std::string v, IntSubset const* c) {
        constraints[graph.getVertex(v)] = c;
    }

    int getRoot(Mate* mate) const {
        for (int k = 0; k < mateSize; ++k) {
            mate[k] = 0;
        }
        mate[mateSize] = 0; // CARP: Indicator of status of tour (0 - undecided, 1 - standard tour, 2 - null tour)

        return n;
    }

    int getChild(Mate* mate, int level, int take) const {
        assert(1 <= level && level <= n);
        int i = n - level;

        if (!take && i % 3 == 1) ++i;       // CARP: Skip 1-edge
        Graph::EdgeInfo const& e = graph.edgeInfo(i);
        
        Mate& w1 = mate[e.v1 - e.v0];
        Mate& w2 = mate[e.v2 - e.v0];
        IntSubset const* c1 = constraints[e.v1];
        IntSubset const* c2 = constraints[e.v2];
        assert(e.v1 <= e.v2);

        if (take) {
            if (i % 3 == 0 && graph.getEdgeDemand(i) == 0) return 0;      // CARP: Take only when needed
            if (!takable(c1, w1, e.v1final, mate[mateSize])) return 0;
            if (!takable(c2, w2, e.v2final, mate[mateSize])) return 0;
            if (c1) ++w1;
            if (c2) ++w2;
            mate[mateSize] = 1;                 // CARP: Standard tour
            if (i % 3 == 0) ++i;                // CARP: Skip 1-edge
        }
        else {
            if (!leavable(c1, w1, e.v1final, mate[mateSize])) return 0;
            if (!leavable(c2, w2, e.v2final, mate[mateSize])) return 0;
        }

        if (e.v1final) w1 = 0;
        if (e.v2final) w2 = 0;

        if (++i == n) return -1;
        shiftMate(mate, graph.edgeInfo(i).v0 - e.v0);
        if (e.finalEdge) mate[mateSize] = 0; // CARP: Reset tour indicator

        while (lookahead) {
            Graph::EdgeInfo const& e = graph.edgeInfo(i);
            Mate& w1 = mate[e.v1 - e.v0];
            Mate& w2 = mate[e.v2 - e.v0];
            IntSubset const* c1 = constraints[e.v1];
            IntSubset const* c2 = constraints[e.v2];
            assert(e.v1 <= e.v2);

            if (takable(c1, w1, e.v1final, mate[mateSize]) && takable(c2, w2, e.v2final, mate[mateSize])) break;
            if (!leavable(c1, w1, e.v1final, mate[mateSize])) return 0;
            if (!leavable(c2, w2, e.v2final, mate[mateSize])) return 0;

            if (e.v1final) w1 = 0;
            if (e.v2final) w2 = 0;

            if (++i == n) return -1;
            shiftMate(mate, graph.edgeInfo(i).v0 - e.v0);
            if (e.finalEdge) mate[mateSize] = 0; // CARP: Reset tour indicator
        }

        assert(i < n);
        return n - i;
    }
};

} // namespace tdzdd
