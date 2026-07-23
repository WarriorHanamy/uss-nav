local modules = {
  MissionFSM = true,
  SceneGraph = true,
  GlobalBelief = true,
  EGOPlanner = true,
  EGOOptimizer = true,
  SUPER = true,
  ExpOpt = true,
  Fsm = true,
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

  -- two-tag form (SUPER style): -- [SUPER][Progress] event=xxx k=v ...
  -- optional leading dashes/spaces are tolerated (" -- [TAG] ..." roscpp prefix style)
  local module, subtag, fields = string.match(message, "^%s*%-*%s*%[([%w_]+)%]%[([%w_]+)%]%s*(.*)$")
  local event
  if module ~= nil then
    event = string.match(fields or "", "event=([^%s]+)")
  else
    -- one-tag form: [Module] event_name k=v ...
    module, event, fields = string.match(message, "^%s*%-*%s*%[([%w_]+)%]%s+([%w_:%-]+)%s*(.*)$")
  end
  if module == nil or modules[module] ~= true then
    return 1, timestamp, record
  end

  record["module"] = module
  if subtag ~= nil then
    record["subtag"] = subtag
  end
  record["event"] = event
  record["event_body"] = fields or ""

  for key, value in string.gmatch(fields or "", "([%w_]+)=([^%s]+)") do
    if record[key] == nil then
      record[key] = convert_value(value)
    end
  end

  return 1, timestamp, record
end
