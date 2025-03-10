strategy opt = maxE(accum_reward + accum_penalty) [<=21]{DroneController.DescisionState,yaw,x,y}->{time} : <> (DroneController.target || time >= 20)
 
saveStrategy("./strategy.json", opt)
 
simulate [<=100;1] {action,accum_reward} : (DroneController.target || time >= 10) under opt