import strategoutil as sutil
from strategoutil import StrategoController
from multiprocessing import Process, Queue
class QueueLengthController(StrategoController):
    def __init__(self, templatefile, state_names):
        super().__init__(templatefile, state_names)
        # variables to insert into the simulation *.xml file
        self.state_names = state_names
         # tag left in model_template.xml
        self.tagRule = "//TAG_{}"

    def insert_state(self, state_dict):
        """
        Uses tag rule to insert state values of [E, S, phase]
        at the appropriate position in the simulation *.xml file
        """
        for name, value in state_dict.items():
            tag = self.tagRule.format(name)
            value = str(value)
            sutil.insert_to_modelfile(
                self.simulation_file, tag, value)
            
    def generate_query_file(self, optimize, learning_param, state_vars, point_vars, observables, horizon):
        strategy = self.generate_strategy_query(optimize, learning_param, state_vars, point_vars, horizon)
        save_strategy = self.generate_store_strategy_query()
        simulate = self.generate_simulate_query(observables, horizon)
        f = open("query.q", "w")
        f.write(strategy +"\n \n" + save_strategy + "\n \n" + simulate)
        f.close()

    def generate_store_strategy_query(self):
        return 'saveStrategy("./strategy.json", opt)'

    def generate_strategy_query(self,optimize, learning_param, state_vars, point_vars, horizon):
        stop_condition = "(DroneController.target || time >= {})".format(horizon)

        strategy_string = "strategy opt = {}({}) [<={}]".format(optimize, learning_param, horizon)
        strategy_string += "{" + ",".join(state_vars) + "}"
        strategy_string += "->"
        strategy_string += "{" + ",".join(point_vars) + "}"
        strategy_string += " : <> " + stop_condition

        return strategy_string
    

    def generate_simulate_query(self, observables, horizon) :
        stop_condition = "(DroneController.target || time >= {})".format(horizon)

        simulate_string = "simulate [<={};1]".format(horizon)
        simulate_string += " {" + ",".join(observables) + "}"
        simulate_string += " : " + stop_condition
        simulate_string += " under opt"

        return simulate_string

    def get_verbose_rewards(self, durations, rewards):
        verbose_rewards = []
        for duration in durations:
            curr_reward = rewards.pop(0)
            for _ in range(duration):
                verbose_rewards.append(curr_reward)

        return verbose_rewards

    def run(self, queryfile="", learning_args={}, verifyta_path="/home/sw9-bois/uppaal-5.0.0-linux64/bin/verifyta", horizon=20):
        print("Enters running Uppaal Model")
        output = super().run(queryfile, learning_args, verifyta_path)
        with open("output.txt", "w") as f:
            f.write(output)
        # parse output

        tpls = sutil.get_float_tuples(output)
        result = sutil.get_duration_action(tpls, max_time=1000)
        d,a = list(zip(*result))

        actions = [int(x) for x in list(a[:horizon])]
        durations = [int(x) for x in list(d[horizon:])]
        rewards = self.get_verbose_rewards(durations, list(a[horizon:]))
        with open("output_parsed.txt", "w") as f:
            f.write(str(actions) + "\n")
            f.write(str(durations) + "\n")
            f.write(str(rewards) + "\n")

        print("Done with running Uppaal Model")
        return actions, rewards

if __name__ == "__main__":
    test_str = """Options for the verification:
  Generating no trace
  Search order is breadth first
  Using conservative space optimisation
  Seed is 1742545597
  State space representation uses minimal constraint systems with future testing
  Using HashMap + Compress integers for discrete state storage
[2K
Verifying formula 1 at /home/gp/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/stompc/query.q:1
 -- Throughput: 87252 runs/sec, Load: 154 runs[K
 -- Throughput: 91910 runs/sec, Load: 299 runs[K
 -- Throughput: 106944 runs/sec, Load: 130 runs[K
 -- Throughput: 90895 runs/sec, Load: 299 runs[K
[2K -- Formula is satisfied.
(800 runs)
Learning statistics for best strategy: 
	Number of resets: 0
	Number of iterations in last reset: 1
	Number of iterations in total: 1


[2K
Verifying formula 2 at /home/gp/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/stompc/query.q:3
[2K -- Formula is satisfied.
$v_gameInfoCounterPlay

[2K
Verifying formula 3 at /home/gp/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/stompc/query.q:5
[2K -- Formula is satisfied.
Tried 1 runs in total and found 1 accepting runs.
action:
[0]: (0,-1) (0,4) (1,4) (1,-1) (1,5) (2,5) (2,-1) (2,5) (3,5) (3,-1) (3,4) (4,4) (4,-1) (4,4) (5,4) (5,-1) (5,5) (6,5) (6,-1) (6,5) (7,5) (7,-1) (7,4) (8,4) (8,-1) (8,4) (9,4) (9,-1) (9,5) (10,5) (10,-1) (10,5) (11,5) (11,-1) (11,4) (12,4) (12,-1) (12,4) (13,4) (13,-1) (13,5) (14,5) (14,-1) (14,5) (15,5) (15,-1) (15,22) (16,22) (16,-1) (16,5) (17,5) (17,-1) (17,23) (18,23) (18,-1) (18,21) (19,21) (19,-1) (19,23) (20,23) (20,-1) (20,21) (21,21) (21,-1) (21,23) (22,23) (22,-1) (22,22) (23,22) (23,-1) (23,22) (24,22) (24,-1) (24,22) (25,22) (25,-1) (25,22) (26,22) (26,-1) (26,22) (27,22) (27,-1) (27,22) (28,22) (28,-1) (28,22) (29,22) (29,-1) (29,20) (30,20) (30,-1) (30,22) (31,22) (31,-1) (31,20) (32,20) (32,-1) (32,22) (33,22) (33,-1) (33,20) (34,20) (34,-1) (34,20) (35,20) (35,-1) (35,20) (36,20) (36,-1) (36,4) (37,4) (37,-1) (37,20) (38,20) (38,-1) (38,20) (39,20) (39,-1) (39,20) (40,20) (40,-1) (40,20) (41,20) (41,-1) (41,20) (42,20) (42,-1) (42,5) (43,5) (43,-1) (43,4) (44,4) (44,-1) (44,5) (45,5) (45,-1) (45,5) (46,5) (46,-1) (46,5) (47,5) (47,-1) (47,5) (48,5) (48,-1) (48,5) (49,5) (49,-1) (49,5) (50,5) (50,-1) (50,5) (51,5) (51,-1) (51,5) (52,5) (52,-1) (52,5) (53,5) (53,-1) (53,5) (54,5) (54,-1) (54,5) (55,5) (55,-1) (55,5) (56,5) (56,-1) (56,5) (57,5) (57,-1) (57,5) (58,5) (58,-1) (58,5) (59,5) (59,-1) (59,5) (60,5) (60,-1) (60,5) (61,5) (61,-1) (61,5) (62,5) (62,-1) (62,22) (63,22) (63,-1) (63,13) (64,13) (64,-1) (64,21) (65,21) (65,-1) (65,4) (66,4) (66,-1) (66,4) (67,4) (67,-1) (67,23) (68,23) (68,-1) (68,22) (69,22) (69,-1) (69,22) (70,22) (70,-1) (70,22) (71,22) (71,-1) (71,22) (72,22) (72,-1) (72,22) (73,22) (73,-1) (73,22) (74,22) (74,-1) (74,22) (75,22) (75,-1) (75,23) (76,23) (76,-1) (76,23) (77,23) (77,-1) (77,23) (78,23) (78,-1) (78,20) (79,20) (79,-1) (79,20) (80,20)
accum_reward:
[0]: (0,0) (0,1000000000) (75,1000000000) (75,1000000074.999999) (76,1000000074.999999) (76,1000000116.55844) (77,1000000116.55844) (77,1000000129.378954) (78,1000000129.378954) (78,1000000216.720725) (79,1000000216.720725) (79,1000000304.220725) (80,1000000304.220725)
"""

    tpls = sutil.get_float_tuples(test_str)
    horizon = 40
    result = sutil.get_duration_action(tpls, max_time=1000)
    d, a = list(zip(*result))

    actions = [int(x) for x in list(a[:horizon])]
    durations = [int(x) for x in list(d[horizon:])]
    # rewards = self.get_verbose_rewards(durations, list(a[horizon:]))

    # def get_verbose_rewards(self, durations, rewards):
    #     verbose_rewards = []
    #     for duration in durations:
    #         curr_reward = rewards.pop(0)
    #         for _ in range(duration):
    #             verbose_rewards.append(curr_reward)
    #
    #     return verbose_rewards

    rewards = list(a[horizon:])
    verbose_rewards = []
    for duration in durations:
        curr_reward = rewards.pop(0)
        for _ in range(duration):
            verbose_rewards.append(curr_reward)

    print("Done!")
