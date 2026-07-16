local modules = {
  MissionFSM = true,
  SceneGraph = true,
  GlobalBelief = true,
  EGOPlanner = true,
  EGOOptimizer = true,
}

local function convert_value(value)
  if value == "true" then
    return true
  end
  if value == "false" then
    return false
  end
  local number = tonumber(value)
  if number ~= nil then
    return number
  end
  return value
end

function extract_roslog_fields(tag, timestamp, record)
  local message = record["message"]
  if message == nil then
    return 1, timestamp, record
  end

  local module, event, fields = string.match(message, "^%[([%w_]+)%]%s+([%w_:%-]+)%s*(.*)$")
  if module == nil or modules[module] ~= true then
    return 1, timestamp, record
  end

  record["module"] = module
  record["event"] = event
  record["event_body"] = fields or ""

  for key, value in string.gmatch(fields or "", "([%w_]+)=([^%s]+)") do
    if record[key] == nil then
      record[key] = convert_value(value)
    end
  end

  return 1, timestamp, record
end
