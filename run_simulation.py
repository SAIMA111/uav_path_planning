import argparse
from uav_simulation.coordinator import Coordinator
from uav_simulation.visualizer import Visualizer

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rows", type=int, default=15)
    parser.add_argument("--cols", type=int, default=15)
    parser.add_argument("--steps", type=int, default=50)
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    start_goals = [((0,0),(args.rows-1,args.cols-1)), ((args.rows-1,0),(0,args.cols-1))]
    coord = Coordinator(rows=args.rows, cols=args.cols, start_goals=start_goals, use_dstar=False, seed=args.seed)
    viz = Visualizer(args.rows, args.cols)

    for step in range(args.steps):
        if coord.all_uavs_at_goal():
            print("All UAVs reached goals!")
            break
        coord.step()
        viz.render(coord.grid, coord.uavs, coord.threat_sim.threats, step)

if __name__ == "__main__":
    main()
