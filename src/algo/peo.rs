use crate::algo::IntoNodeIdentifiers;
use crate::algo::NodeIndexable;
use crate::algo::Visitable;
use crate::visit::IntoEdges;
use std::collections::HashSet;
use std::hash::Hash;

/// try to find a peo
/// The input graph is treated as if undirected.
pub fn peo<G>(graph: &G) -> Option<Vec<G::NodeId>>
where
    G: Visitable + NodeIndexable + IntoNodeIdentifiers + IntoEdges,
    G::NodeId: Hash + Eq,
{
    let mut v: Vec<G::NodeId> = vec![];
    let mut nodes: HashSet<G::NodeId> = graph.node_identifiers().collect();

    'outer: while !nodes.is_empty() {
        for a in nodes.iter().copied() {
            if is_clique(graph, graph.neighbors(a).collect()) {
                v.push(a);
                nodes.remove(&a);
                continue 'outer;
            }
        }
        return None;
    }

    Some(v)
}

/// The input graph is treated as if undirected.
fn is_clique<G>(graph: G, nodes: HashSet<G::NodeId>) -> bool
where
    G: Visitable + NodeIndexable + IntoNodeIdentifiers + IntoEdges,
    G::NodeId: Hash + Eq,
{
    for a in &nodes {
        let mut y = nodes.clone();
        y.remove(a);
        for b in graph.neighbors(*a) {
            y.remove(&b);
        }
        if !y.is_empty() {
            return false;
        }
    }
    true
}
