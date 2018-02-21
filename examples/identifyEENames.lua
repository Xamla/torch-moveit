local ros = require 'ros'
local moveit = require 'moveit'

ros.init('robot_trajectory')
local sp = ros.AsyncSpinner()
sp:start()

local robot_model_loader = moveit.RobotModelLoader('robot_description')
local kinematic_model = robot_model_loader:getModel()

print(kinematic_model:printModelInfo())
local overview_move_group_names, overview_group_link_names = kinematic_model:getEndEffectorParentGroups()
print(overview_move_group_names)
print(overview_group_link_names)
os.exit()
local group_names = kinematic_model:getJointModelGroupNames()
print('group_names', table.unpack(group_names:totable()))
local end_effector_names = kinematic_model:getEndEffectorNames()
print('end_effector_names', table.unpack(end_effector_names:totable() or ''))
for i, v in ipairs(group_names:totable()) do
  print(i, v)
  local tip_names = kinematic_model:getGroupEndEffectorTipNames(v):totable()
  local attached_names = kinematic_model:getAttachedEndEffectorNames(v):totable()
  print('GroupName:', v, string.format('\tAttachedEndEffectorNames'), table.unpack(attached_names) or '', 'tipNames: ', table.unpack(tip_names))
  local name, suc = kinematic_model:getGroupEndEffectorName(v)
  if suc then

    if name == '' and #attached_names > 0 then
      name = attached_names[1]
    end
    if name ~= '' then
      local link_name, suc = kinematic_model:getEndEffectorLinkName(name)
      if suc then
        print('GroupName:', v, 'EndEffectorName', name, 'LinkName:', link_name)
      end
    end
  end
end

ros.shutdown()
