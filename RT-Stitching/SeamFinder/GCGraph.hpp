// =============================================================================
// GCGraph.hpp
// -----------------------------------------------------------------------------
// Boykov-Kolmogorov max-flow / min-cut implementation, header-only.
//
// Reproduced from OpenCV's internal cv::detail::GCGraph (modules/imgproc/src/
// gcgraph.hpp), which is BSD-3-clause licensed. Placed in our own namespace to
// avoid ODR conflicts when linking against OpenCV.
//
// Reference:
//   Yuri Boykov, Vladimir Kolmogorov,
//   "An Experimental Comparison of Min-Cut/Max-Flow Algorithms for Energy
//    Minimization in Vision", PAMI 2004.
//
// Usage:
//   gaze_seam::GCGraph<float> g(num_vertices, num_edges);
//   for (...) { int v = g.addVtx(); g.addTermWeights(v, src_w, sink_w); }
//   for (...) g.addEdges(i, j, w_ij, w_ji);
//   g.maxFlow();
//   bool in_source = g.inSourceSegment(v);
// =============================================================================
#pragma once

#include <vector>
#include <climits>
#include <cmath>
#include <algorithm>

namespace gaze_seam {

template <class TWeight>
class GCGraph {
public:
    GCGraph() : flow_(0) {}
    GCGraph(unsigned int vtx_count, unsigned int edge_count) { create(vtx_count, edge_count); }
    ~GCGraph() = default;

    void create(unsigned int vtx_count, unsigned int edge_count) {
        vtcs_.clear();
        edges_.clear();
        vtcs_.reserve(vtx_count);
        edges_.reserve(edge_count + 2);
        flow_ = 0;
    }

    /// Add a new vertex; returns its index.
    int addVtx() {
        Vtx v{};
        vtcs_.push_back(v);
        return static_cast<int>(vtcs_.size()) - 1;
    }

    /// Add an undirected edge with possibly asymmetric capacities.
    /// Forward and reverse edges are stored at consecutive indices so that the
    /// XOR trick `e ^ 1` toggles between them.
    void addEdges(int i, int j, TWeight w, TWeight rev_w) {
        // Reserve slots 0/1 as the "null" sentinel (next == 0 means no more).
        if (edges_.empty())
            edges_.resize(2);

        Edge from_i{};
        from_i.dst    = j;
        from_i.next   = vtcs_[i].first;
        from_i.weight = w;
        vtcs_[i].first = static_cast<int>(edges_.size());
        edges_.push_back(from_i);

        Edge to_i{};
        to_i.dst    = i;
        to_i.next   = vtcs_[j].first;
        to_i.weight = rev_w;
        vtcs_[j].first = static_cast<int>(edges_.size());
        edges_.push_back(to_i);
    }

    /// Set source / sink terminal capacities for vertex i.
    /// Multiple calls accumulate.
    void addTermWeights(int i, TWeight source_w, TWeight sink_w) {
        TWeight dw = vtcs_[i].weight;
        if (dw > 0) source_w += dw;
        else        sink_w   -= dw;
        flow_ += (source_w < sink_w) ? source_w : sink_w;
        vtcs_[i].weight = source_w - sink_w;
    }

    /// Run BK max-flow. Returns the value of the maximum flow.
    TWeight maxFlow();

    /// After maxFlow(), returns true if vertex i is on the source side of the cut.
    bool inSourceSegment(int i) const { return vtcs_[i].t == 0; }

private:
    struct Vtx {
        Vtx*           next;     // active queue link
        int            parent;   // parent edge id (TERMINAL/ORPHAN/-)
        int            first;    // first outgoing edge id
        int            ts;       // timestamp
        int            dist;     // distance to terminal
        TWeight        weight;   // residual capacity to terminal
        unsigned char  t;        // tree affiliation: 0 = source, 1 = sink
    };
    struct Edge {
        int     dst;     // destination vertex
        int     next;    // next edge from same source vertex
        TWeight weight;  // residual capacity
    };

    std::vector<Vtx>  vtcs_;
    std::vector<Edge> edges_;
    TWeight           flow_;
};

// ---------------------------------------------------------------------------
// BK max-flow main loop: growth -> augmentation -> adoption.
// ---------------------------------------------------------------------------
template <class TWeight>
TWeight GCGraph<TWeight>::maxFlow() {
    constexpr int TERMINAL = -1;
    constexpr int ORPHAN   = -2;

    if (vtcs_.empty()) return flow_;

    Vtx  stub{};
    stub.next = &stub;
    Vtx* nil_node = &stub;
    Vtx* first    = nil_node;
    Vtx* last     = nil_node;
    int  curr_ts  = 0;

    Vtx*  vtx_ptr  = vtcs_.data();
    Edge* edge_ptr = edges_.empty() ? nullptr : edges_.data();

    std::vector<Vtx*> orphans;

    // ---- Initialize active queue ----
    for (int i = 0; i < static_cast<int>(vtcs_.size()); ++i) {
        Vtx* v = vtx_ptr + i;
        v->ts = 0;
        if (v->weight != 0) {
            last = last->next = v;
            v->dist   = 1;
            v->parent = TERMINAL;
            v->t      = (v->weight < 0) ? 1 : 0;
        } else {
            v->parent = 0;
        }
    }
    first         = first->next;
    last->next    = nil_node;
    nil_node->next = nullptr;

    // ---- Main BK loop ----
    for (;;) {
        Vtx* v = nullptr;
        Vtx* u = nullptr;
        int  e0 = -1, ei = 0, ej = 0;
        TWeight       min_weight, weight;
        unsigned char vt;

        // ----- GROWTH: expand S/T trees, find a connecting edge -----
        while (first != nil_node) {
            v = first;
            if (v->parent) {
                vt = v->t;
                for (ei = v->first; ei != 0; ei = edge_ptr[ei].next) {
                    if (edge_ptr[ei ^ vt].weight == 0) continue;
                    u = vtx_ptr + edge_ptr[ei].dst;
                    if (!u->parent) {
                        u->t      = vt;
                        u->parent = ei ^ 1;
                        u->ts     = v->ts;
                        u->dist   = v->dist + 1;
                        if (!u->next) {
                            u->next = nil_node;
                            last = last->next = u;
                        }
                        continue;
                    }
                    if (u->t != vt) {
                        e0 = ei ^ vt;
                        break;
                    }
                    if (u->dist > v->dist + 1 && u->ts <= v->ts) {
                        u->parent = ei ^ 1;
                        u->ts     = v->ts;
                        u->dist   = v->dist + 1;
                    }
                }
                if (e0 > 0) break;
            }
            first = first->next;
            v->next = nullptr;
        }

        if (e0 <= 0) break;  // no augmenting path -> done

        // ----- AUGMENTATION: walk the path, find bottleneck -----
        min_weight = edge_ptr[e0].weight;
        for (int k = 1; k >= 0; --k) {
            for (v = vtx_ptr + edge_ptr[e0 ^ k].dst; ;
                 v = vtx_ptr + edge_ptr[ei].dst)
            {
                if ((ei = v->parent) < 0) break;
                weight     = edge_ptr[ei ^ k].weight;
                min_weight = std::min(min_weight, weight);
            }
            weight     = static_cast<TWeight>(std::abs(static_cast<double>(v->weight)));
            min_weight = std::min(min_weight, weight);
        }

        // Push flow along the path.
        edge_ptr[e0    ].weight -= min_weight;
        edge_ptr[e0 ^ 1].weight += min_weight;
        flow_ += min_weight;

        for (int k = 1; k >= 0; --k) {
            for (v = vtx_ptr + edge_ptr[e0 ^ k].dst; ;
                 v = vtx_ptr + edge_ptr[ei].dst)
            {
                if ((ei = v->parent) < 0) break;
                edge_ptr[ei ^ (k ^ 1)].weight += min_weight;
                if ((edge_ptr[ei ^ k].weight -= min_weight) == 0) {
                    orphans.push_back(v);
                    v->parent = ORPHAN;
                }
            }
            v->weight = v->weight + min_weight * (1 - k * 2);
            if (v->weight == 0) {
                orphans.push_back(v);
                v->parent = ORPHAN;
            }
        }

        // ----- ADOPTION: re-parent orphans -----
        ++curr_ts;
        while (!orphans.empty()) {
            Vtx* v2 = orphans.back();
            orphans.pop_back();

            int d, min_dist = INT_MAX;
            e0 = 0;
            vt = v2->t;

            for (ei = v2->first; ei != 0; ei = edge_ptr[ei].next) {
                if (edge_ptr[ei ^ (vt ^ 1)].weight == 0) continue;
                u = vtx_ptr + edge_ptr[ei].dst;
                if (u->t != vt || u->parent == 0) continue;

                // Compute distance from u to its tree root.
                for (d = 0; ;) {
                    if (u->ts == curr_ts) { d += u->dist; break; }
                    ej = u->parent;
                    ++d;
                    if (ej < 0) {
                        if (ej == ORPHAN) {
                            d = INT_MAX - 1;
                        } else {
                            u->ts   = curr_ts;
                            u->dist = 1;
                        }
                        break;
                    }
                    u = vtx_ptr + edge_ptr[ej].dst;
                }

                if (++d < INT_MAX) {
                    if (d < min_dist) { min_dist = d; e0 = ei; }
                    for (u = vtx_ptr + edge_ptr[ei].dst;
                         u->ts != curr_ts;
                         u = vtx_ptr + edge_ptr[u->parent].dst)
                    {
                        u->ts   = curr_ts;
                        u->dist = --d;
                    }
                }
            }

            if ((v2->parent = e0) > 0) {
                v2->ts   = curr_ts;
                v2->dist = min_dist;
                continue;
            }

            // Truly orphaned: scan neighbours, may activate them.
            v2->ts = 0;
            for (ei = v2->first; ei != 0; ei = edge_ptr[ei].next) {
                u = vtx_ptr + edge_ptr[ei].dst;
                ej = u->parent;
                if (u->t != vt || !ej) continue;
                if (edge_ptr[ei ^ (vt ^ 1)].weight && !u->next) {
                    u->next = nil_node;
                    last = last->next = u;
                }
                if (ej > 0 && vtx_ptr + edge_ptr[ej].dst == v2) {
                    orphans.push_back(u);
                    u->parent = ORPHAN;
                }
            }
        }
    }

    return flow_;
}

}  // namespace gaze_seam
