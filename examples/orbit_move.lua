moveit = require 'moveit'
ros = moveit.ros
tf = moveit.tf

--[[

Comupte points along an arc on a plane around a center point and normal.

You can use rviz to display the moving 'eye' transform with the TF display.
Please make sure 'Fixed Frame' is set to 'world'.

]]

function normalize(v)
  return v / torch.norm(v)
end

function totensor(t)
  return not t or torch.isTensor(t) and t or torch.Tensor(t)
end

function pos_vector(v)
  if v:size(1) ~= 4 then
    v = torch.Tensor({ v[1], v[2], v[3], 1 })
  end
  return v
end

function project_onto_plane(plane_point, plane_normal, pt)
  return pt - plane_normal * torch.dot(pt - plane_point, plane_normal)
end

function look_at_pose(eye, at, up)
  -- eye becomes origin, 'at' lies on x-axis
  local zaxis = normalize(at - eye)
  local xaxis = -normalize(torch.cross(zaxis, up))
  local yaxis = torch.cross(zaxis, xaxis)

  local basis = torch.Tensor(3,3)
  basis[{{},{1}}] = xaxis
  basis[{{},{2}}] = yaxis
  basis[{{},{3}}] = zaxis

  local t = tf.Transform()
  t:setBasis(basis)
  t:setOrigin(eye)
  
  local rot90z = tf:Transform()
  rot90z:setRotation(tf.Quaternion({0,0,1}, -0.5 * math.pi))
  t:mul(rot90z, t)
  
  return t
end

function generate_arc(center, normal, start_pt, total_rotation_angle, angle_step, look_at, up)
  up = up or torch.Tensor({0,0,1})
  look_at = look_at or center

  center = totensor(center)
  normal = totensor(normal)
  start_pt = totensor(start_pt)
  start_pt = project_onto_plane(center, normal, start_pt)
  look_at = totensor(look_at)
  up = totensor(up)

  local poses = {}
  local steps = math.max(math.floor(total_rotation_angle / angle_step + 0.5), 1)
  for i=0,steps do
    local theta = total_rotation_angle * i / steps

    local t = tf.Transform()
    t:setRotation(tf.Quaternion(normal, theta))
    t:setOrigin(center)

    local eye = t:toTensor() * pos_vector(start_pt-center)
    local pose = look_at_pose(eye[{{1,3}}], look_at, up)
    table.insert(poses, pose)
  end

  return poses
end

x = generate_arc({0.0,0.7,0.6}, {0,0,1}, {-0.4,0.4,0.3}, 0.5 * math.pi, 0.1, {0.0,0.7,0.0})

ros.init('lookat')
ros.Time.init()

local b = tf.TransformBroadcaster()

ps = moveit.PlanningSceneInterface()
ps:addPlane('ground plane', 0, 0, 1, -0.001)

g = moveit.MoveGroup('arm')

local pose = g:getCurrentPose_StampedTransform()
print('pose:'..tostring(pose:getOrigin())..tostring(pose:getRotation():getRPY()))
g:setMaxVelocityScalingFactor(0.1)

local sp = ros.AsyncSpinner()
sp:start()

local i = 1
while ros.ok() do

  local pose = x[i]

  g:setPoseReferenceFrame('/world')

  print('goal pose:')
  print(pose:toTensor())
  
  g:setStartStateToCurrentState()
  g:setPoseTarget(pose:toTensor())
  s, p = g:plan()
  if s then
    g:execute(p)
  end

  ros.Duration(0.5):sleep()
  i = (i % #x) + 1
end
