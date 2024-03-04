MultiSwarm Cooperative Information-driven search and Divide and Conquer mitigation control(MSCIDC)
This repository contains MultiSwarm Cooperative Information-driven search and Divide and Conquer 
mitigation control(MSCIDC) framework for multi-swarm systems, to efficiently search and neutralize
dynamic target (forest fire) in an unknown/uncertain environment.

The script implements work reported in

J. John, K. Harikumar, J. Senthilnath and S. Sundaram, "An Efficient Approach With Dynamic Multiswarm of UAVs for Forest Firefighting," 
in IEEE Transactions on Systems, Man, and Cybernetics: Systems, doi: 10.1109/TSMC.2024.3352660.


Matlab tested version R2022b

List of files supporting MSCIDCmain.m (main file) are:
1) levy.m contains levy distribution
2) uavdynamicsvelocity.m contains UAV dynamics
3) ellipseang.m computes the initial angular position for divide and conquer mitigation
4) refpoint.m computes the reference point if the point crosses the swarm boundary
5) searchopp.m computes the reference points for search in opposite direction of maximum information

How to execute:
1) Use matlab version mentioned above as this software is tested
2) Copy all the six matlab files into single folder
3) Execute MSCIDCmain.m
