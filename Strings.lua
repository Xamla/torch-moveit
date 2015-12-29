local ffi = require 'ffi'
local torch = require 'torch'
local moveit = require 'moveit.env'
local utils = require 'moveit.utils'

local Strings = torch.class('moveit.Strings', moveit)

local f

function init()
  local Strings_method_names = {
    "new",
    "clone",
    "delete",
    "size",
    "getAt",
    "setAt",
    "push_back",
    "pop_back",
    "clear",
    "insert",
    "erase",
    "empty"
  }
  
  f = utils.create_method_table("moveit_Strings_", Strings_method_names )
end

init()

function Strings:__init(...)
  rawset(self, 'o', f.new())
  if select("#", ...) > 0 then
    local x = ...
    if type(x) ~= 'table' then
      x = { ... }
    end
    self:insertFromTable(1, x)
  end
end

function Strings:cdata()
  return self.o
end

function Strings:clone()
  local c = torch.factory('moveit.Strings')()
  rawset(c, 'o', f.clone(self.o))
  return c
end

function Strings:size()
  return f.size(self.o)
end

function Strings:__len()
  return self:size()
end

function Strings:__index(idx)
  local v = rawget(self, idx)
  if not v then 
    v = Strings[idx]
    if not v and type(idx) == 'number' then
      local o = rawget(self, 'o')
      v = ffi.string(f.getAt(o, idx-1))
    end
  end
  return v
end

function Strings:__newindex(idx, v)
  local o = rawget(self, 'o')
  if type(idx) == 'number' then
    f.setAt(o, idx-1, tostring(v))
  else
    rawset(self, idx, v)
  end
end

function Strings:push_back(value)
  f.push_back(self.o, tostring(value))
end

function Strings:pop_back()
  local last = self[#self]
  f.pop_back(self.o)
  return last
end

function Strings:clear()
  f.clear(self.o)
end

function Strings:insert(pos, value, n)
  if pos < 1 then
    pos = 1
  elseif pos > #self+1 then
    pos = #self + 1
  end
  f.insert(self.o, pos-1, n or 1, value)
end

function Strings:insertFromTable(pos, t)
  if type(pos) == 'table' then
    t = pos
    pos = #self + 1
  end
  pos = pos or #self + 1
  for _,v in pairs(t) do
    self:insert(pos, v)
    pos = pos + 1
  end
end

function Strings:erase(begin_pos, end_pos)
  f.erase(self.o, begin_pos-1, (end_pos or begin_pos + 1)-1)
end

function Strings:__pairs()
  return function (t, k)
    local i = k or 1
    if i > #t then
      return nil
    else
      local v = t[i]
      return i+1, v
    end
  end, self, nil
end

function Strings:__ipairs()
  return self:__pairs()
end

function Strings:totable()
  local t = {}
  for i,v in ipairs(self) do
    table.insert(t, v)
  end
  return t
end

function Strings:__tostring()
  local t = self:totable()
  return table.concat(t, '\n')
end
