#![cfg(feature = "quickcheck")]
use petgraph::algo::bidirectional_dijkstra::bidirectional_dijkstra;
use petgraph::algo::dijkstra;
use petgraph::Graph;
use quickcheck::{Arbitrary, StdThreadGen};

#[test]
fn bidirectional_dijkstra_correctness() {
    let mut gen = StdThreadGen::new(100);
    let mut graph: Graph<(), u64> = Graph::arbitrary(&mut gen);

    for weight in graph.edge_weights_mut() {
        if *weight == 0 {
            *weight = 1;
        }
    }
    let graph = graph;

    let mut nodes = graph.node_indices();
    let start = nodes.next().unwrap();

    let correct = dijkstra(&graph, start, None, |e| *e.weight());
    for (goal, res) in correct {
        if goal != start {
            let y = bidirectional_dijkstra(&graph, start, goal, |e| *e.weight());
            assert_eq!(y, res);
        }
    }
}
