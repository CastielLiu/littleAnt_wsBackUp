//when setting the vehicle into driverless mode
//the steering will be back to the middle automaticly, and the steer rotate speed is suitable
//but if we immdiately send the steering cmd, the steer will rotate very fast, even cause EPS to fail
//so we should wait a minute before we send steering cmd
//if we just use sleep(x) but not keep cmd being sending ,the driverless system in vehicle may exit
