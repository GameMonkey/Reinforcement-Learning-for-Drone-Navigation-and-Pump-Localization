strategy opt = maxE(accum_reward + accum_penalty) [<=25]{DroneController.DescisionState,yaw,x,y}->{time} : <> (DroneController.target || time >= 25)
 
saveStrategy("./strategy.json", opt)
 
simulate [<=25;1] {action,accum_reward} : (DroneController.target || time >= 25) under opt