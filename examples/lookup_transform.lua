moveit = require 'moveit'
ros = moveit.ros
tf = moveit.tf

ros.init('example_TransformListener', 0)

local sp = ros.AsyncSpinner()
sp:start()

l = tf.TransformListener()

ros.Time.init()



for i=1,10 do

  print(ros.Time.now())

  local s, e = l:waitForTransform('world', 'tomato', ros.Time(0), ros.Duration(10))
  print(e)

  transform = tf.StampedTransform()
  l:lookupTransform('world', 'tomato', ros.Time(0), transform)
  print(transform)

  ros.Duration(1):sleep()
end
