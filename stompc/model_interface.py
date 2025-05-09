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
        output = super().run(queryfile, learning_args, verifyta_path)
        # with open("output.txt", "w") as f:
        #     f.write(output)
        # parse output

        tpls = sutil.get_float_tuples(output)
        result = sutil.get_duration_action(tpls, max_time=1000)
        d,a = list(zip(*result))

        actions = [int(x) for x in list(a[:horizon])]
        durations = [int(x) for x in list(d[horizon:])]
        rewards = self.get_verbose_rewards(durations, list(a[horizon:]))


        return actions, rewards