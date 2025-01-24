strategy opt = maxE(accum_reward) [<=21]{DroneController.DescisionState}->{yaw,x,y} : <> (DroneController.target || time >= 20)
 
saveStrategy("./strategy.json", opt)
 
simulate [<=100;1] {action} : (DroneController.target || time >= 10) under opt