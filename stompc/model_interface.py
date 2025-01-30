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
            
    def generate_query_file(self, optimize, learning_param, state_vars, point_vars, observables):
        strategy = self.generate_strategy_query(optimize, learning_param, state_vars, point_vars)
        save_strategy = self.generate_store_strategy_query()
        simulate = self.generate_simulate_query(observables)
        f = open("query.q", "w")
        f.write(strategy +"\n \n" + save_strategy + "\n \n" + simulate)
        f.close()

    def generate_store_strategy_query(self):
        return 'saveStrategy("./strategy.json", opt)'

    def generate_strategy_query(self,optimize, learning_param, state_vars, point_vars):

        time_to_reach_stop_condition = "21"
        
        stop_condition = "(DroneController.target || time >= 20)"

        strategy_string = "strategy opt = {}({}) [<={}]".format(optimize, learning_param, time_to_reach_stop_condition)
        strategy_string += "{" + ",".join(state_vars) + "}"
        strategy_string += "->"
        strategy_string += "{" + ",".join(point_vars) + "}"
        strategy_string += " : <> " + stop_condition

        return strategy_string
    

    def generate_simulate_query(self, observables) :
        simulate_length = "100"
        stop_condition = "(DroneController.target || time >= 10)"

        simulate_string = "simulate [<={};1]".format(simulate_length)
        simulate_string += " {" + ",".join(observables) + "}"
        simulate_string += " : " + stop_condition
        simulate_string += " under opt"

        return simulate_string

    def run(self, queryfile="", learning_args={}, verifyta_path="/home/sw9-bois/uppaal-5.0.0-linux64/bin/verifyta"):
        output = super().run(queryfile, learning_args, verifyta_path)
        # parse output
   
        tpls = sutil.get_int_tuples(output)
        result = sutil.get_duration_action(tpls, max_time=1000)
        d,a = list(zip(*result))
        #actions.send(a)
        #actions.close()
        return list(a)