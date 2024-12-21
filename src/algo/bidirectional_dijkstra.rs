use std::cmp::min;
use std::collections::hash_map::Entry::{Occupied, Vacant};
use std::collections::{BinaryHeap, HashMap};
use std::hash::Hash;

use crate::algo::PositiveMeasure;
use crate::scored::MinScored;
use crate::visit::{EdgeRef, IntoEdgesDirected, VisitMap, Visitable};
use crate::EdgeDirection::Incoming;
use crate::EdgeDirection::Outgoing;

/// https://arxiv.org/pdf/2410.14638
/// \[Generic\] Dijkstra's shortest path algorithm.
///
/// Compute the length of the shortest path from `start` to `goal` node.
///
/// The graph should be `Visitable` and implement `IntoEdges`. The function
/// `edge_cost` should return the cost for a particular edge, which is used
/// to compute path costs. Edge costs must be non-negative.
///
/// Returns a `HashMap` that maps `NodeId` to path cost.
/// # Example
/// ```rust
/// use petgraph::Graph;
/// use petgraph::algo::dijkstra;
/// use petgraph::prelude::*;
/// use std::collections::HashMap;
///
/// let mut graph: Graph<(), (), Directed> = Graph::new();
/// let a = graph.add_node(()); // node with no weight
/// let b = graph.add_node(());
/// let c = graph.add_node(());
/// let d = graph.add_node(());
/// let e = graph.add_node(());
/// let f = graph.add_node(());
/// let g = graph.add_node(());
/// let h = graph.add_node(());
/// // z will be in another connected component
/// let z = graph.add_node(());
///
/// graph.extend_with_edges(&[
///     (a, b),
///     (b, c),
///     (c, d),
///     (d, a),
///     (e, f),
///     (b, e),
///     (f, g),
///     (g, h),
///     (h, e),
/// ]);
/// // a ----> b ----> e ----> f
/// // ^       |       ^       |
/// // |       v       |       v
/// // d <---- c       h <---- g
///
/// let expected_res: HashMap<NodeIndex, usize> = [
///     (a, 3),
///     (b, 0),
///     (c, 1),
///     (d, 2),
///     (e, 1),
///     (f, 2),
///     (g, 3),
///     (h, 4),
/// ].iter().cloned().collect();
/// let res = dijkstra(&graph, b, None, |_| 1);
/// assert_eq!(res, expected_res);
/// // z is not inside res because there is not path from b to z.
/// ```
pub fn bidirectional_dijkstra<G, F, K>(
    graph: G,
    start: G::NodeId,
    goal: G::NodeId,
    mut edge_cost: F,
) -> K
where
    G: IntoEdgesDirected + Visitable,
    G::NodeId: Eq + Hash,
    F: FnMut(G::EdgeRef) -> K,
    K: PositiveMeasure + Copy + std::cmp::Ord,
{
    let mut μ: K = PositiveMeasure::max();

    let zero_score = K::default();

    let mut scores = HashMap::new();
    scores.insert((start, start), zero_score);
    scores.insert((goal, goal), zero_score);

    let mut us = start;
    let mut f_closed = graph.visit_map();
    f_closed.visit(us);
    let mut f_edge_iter = graph.edges_directed(us, Outgoing);

    let mut ut = goal;
    let mut b_closed = graph.visit_map();
    b_closed.visit(ut);
    let mut b_edge_iter = graph.edges_directed(ut, Incoming);

    // let mut visited = graph.visit_map();
    //let mut predecessor = HashMap::new();
    let mut f_visit_next = BinaryHeap::new();
    f_visit_next.push(MinScored(zero_score, start));

    let mut b_visit_next = BinaryHeap::new();
    b_visit_next.push(MinScored(zero_score, goal));

    'outer: loop {
        'forward: loop {
            if scores[&(start, us)] + scores[&(ut, goal)] >= μ {
                break 'outer;
            }
            if let Some(edge) = f_edge_iter.next() {
                // relax
                let next = edge.target();
                if f_closed.is_visited(&next) {
                    break 'forward;
                }
                let next_score = scores[&(start, us)] + edge_cost(edge);
                match scores.entry((start, next)) {
                    Occupied(ent) => {
                        if next_score < *ent.get() {
                            *ent.into_mut() = next_score;
                            f_visit_next.push(MinScored(next_score, next));
                            //predecessor.insert(next.clone(), node.clone());
                        }
                    }
                    Vacant(ent) => {
                        ent.insert(next_score);
                        f_visit_next.push(MinScored(next_score, next));
                        //predecessor.insert(next.clone(), node.clone());
                    }
                }
                if b_closed.is_visited(&next) {
                    μ = min(
                        μ,
                        scores[&(start, us)] + edge_cost(edge) + scores[&(next, goal)],
                    );
                }
            } else {
                loop {
                    match f_visit_next.pop() {
                        Some(MinScored(_node_score, node)) => {
                            if !f_closed.is_visited(&node) {
                                f_closed.visit(node);
                                us = node;
                                f_edge_iter = graph.edges_directed(us, Outgoing);
                                continue 'forward;
                            }
                        }
                        None => break 'outer,
                    }
                }
            }
        }

        'backward: loop {
            if scores[&(start, us)] + scores[&(ut, goal)] >= μ {
                break 'outer;
            }
            if let Some(edge) = b_edge_iter.next() {
                // relax
                let next = edge.source();
                if b_closed.is_visited(&next) {
                    break 'backward;
                }
                let next_score = scores[&(ut, goal)] + edge_cost(edge);
                match scores.entry((next, goal)) {
                    Occupied(ent) => {
                        if next_score < *ent.get() {
                            *ent.into_mut() = next_score;
                            b_visit_next.push(MinScored(next_score, next));
                            //predecessor.insert(next.clone(), node.clone());
                        }
                    }
                    Vacant(ent) => {
                        ent.insert(next_score);
                        b_visit_next.push(MinScored(next_score, next));
                        //predecessor.insert(next.clone(), node.clone());
                    }
                }
                if f_closed.is_visited(&next) {
                    μ = min(
                        μ,
                        scores[&(start, next)] + edge_cost(edge) + scores[&(ut, goal)],
                    );
                }
            } else {
                loop {
                    match b_visit_next.pop() {
                        Some(MinScored(_node_score, node)) => {
                            if !b_closed.is_visited(&node) {
                                b_closed.visit(node);
                                ut = node;
                                b_edge_iter = graph.edges_directed(ut, Incoming);
                                continue 'backward;
                            }
                        }
                        None => break 'outer,
                    }
                }
            }
        }
    }

    μ
}
