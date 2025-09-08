#ICBS 소스에서 import
from cbs_basic import CBSSolver
from icbs_cardinal_bypass import ICBS_CB_Solver
from icbs_complete import ICBS_Solver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

class CBSManager:
    def __init__(self, solver_type="ICBS", disjoint=False, visualize_result=True):
        self.solver_type = solver_type
        self.disjoint = disjoint
        self.visualize_result = visualize_result
        self.agents = []

    def load_instance(self, my_map, agents):
        self.my_map = my_map
        self.agents = agents

    def create_solver(self):
        if self.solver_type == "CBS":
            return CBSSolver(self.my_map, self.agents)
        elif self.solver_type == "ICBS_CB":
            return ICBS_CB_Solver(self.my_map, self.agents)
        elif self.solver_type == "ICBS":
            return ICBS_Solver(self.my_map, self.agents)
        else:
            raise ValueError(f"Unknown solver type: {self.solver_type}")

    def run(self):
        solver = self.create_solver()
        result = solver.find_solution(self.disjoint)

        if result is None:
            print("No solution found.")
            return None

        paths, nodes_generated, nodes_expanded = result

        for agent, path in zip(self.agents, paths):
            agent.set_path(path)

        print(f"Total cost: {get_sum_of_cost(paths)}")
        print(f"Nodes generated: {nodes_generated}, Nodes expanded: {nodes_expanded}")

        if self.visualize_result:
            animation = Animation(self.my_map,
                                  [agent.start for agent in self.agents],
                                  [agent.goal for agent in self.agents],
                                  [agent.get_final_path() for agent in self.agents])
            animation.show()

        return [agent.path for agent in self.agents]

    def get_agents(self):
        return self.agents