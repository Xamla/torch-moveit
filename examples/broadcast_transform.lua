moveit = require 'moveit'
ros = moveit.ros
tf = moveit.tf

ros.init('example_TransformBroadcaster')
local sp = ros.AsyncSpinner()
sp:start()
print('hallo')
b = tf.TransformBroadcaster()

--[[ 

tf_echo can be used to verify that the transform is correctly broadcasted:

rosrun tf tf_echo /world /tomato

]]

for i=1,10 do
  local t = tf.Transform()
  t:setOrigin({ i, -i, i*2 })
  local rot = tf.Quaternion()
  rot:setRPY(i*10, i*20, i*30, true)  -- last argument indicates that angle is specified in degrees
  t:setRotation(rot)
  
  local st = tf.StampedTransform(t, ros.Time.now(), 'world', 'tomato')
  
  print('Sending:')
  print(st)
  
  b:sendTransform(st)
  ros.Duration(1):sleep()
end
