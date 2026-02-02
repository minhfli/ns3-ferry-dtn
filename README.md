# This is for my graduation research project

The problem: Using UAV as data ferry to help re-establish communcation for stationary nodes in post-disaster scenarios (flooding to be exact).

## Code:

- main file: ferry.cc
- helper files, mainly for implementing algorithms to solve TSP, clustering, visualize, etc.

How to run: Copy ferry.cc file and ferry_helper to ns3 scratch folder then ./waf --run scratch.cc

Note: tested and works with ns3.30.1, disable -Werror


## References

The packet sending flow (what packet to send, logic which handle it, ...), took ideas from:
- Naive implementation of epidemic and prophet
    - https://github.com/SantinoI/DTN-Networks-ns3
- Helpful guide, but no code
    - https://dl.acm.org/doi/10.1145/2756509.2756523
- Helpful guild pt2, but still no code :( 
    - https://arxiv.org/abs/1805.10539

Previous research
- (2018) https://ieeexplore.ieee.org/document/8566956
- (2018) https://link.springer.com/article/10.1007/s11036-018-1038-7