local statistic_functions = {}
function statistic_functions.median(data)
    table.sort(data)
    return data[math.ceil(#data / 2)]
end

function statistic_functions.average(data)
    local sum = 0
    for _, v in pairs(data) do sum = sum + v end
    return sum / #data
end

function statistic_functions.stddev(data)
    local mean = statistic_functions.average(data)

    local sum = 0
    for _, value in pairs(data) do
        assert(type(value) == "number")
        local delta = value - mean
        sum = sum + (delta * delta)
    end

    return math.sqrt(sum / (#data - 1))
end

function statistic_functions.min(data)
    local min = data[1]
    for _, value in pairs(data) do
        assert(type(value) == "number")
        if value < min then min = value end
    end
    return min
end

function statistic_functions.max(data)
    local max = data[1]
    for _, value in pairs(data) do
        assert(type(value) == "number")
        if value > max then max = value end
    end
    return max
end

local file = io.open("FRC_UserProgram.log")
if not file then return end
data = {}
for line in file:lines() do
    name, measure = line:match("([a-zA-Z.()]+): (%d.%d%d%d%d%d%d)s")
    if name or measure then
        data[name] = data[name] or {}
        table.insert(data[name], measure * 1000)
    end
end

local averages = {}
for name, data in pairs(data) do averages[name] = statistic_functions.average(data) end
local indexes = {}
for name, _ in pairs(averages) do table.insert(indexes, name) end
table.sort(indexes, function (k1, k2) return averages[k1] < averages[k2] end)

for _, i in ipairs(indexes) do
    print(string.format("%7.3f\t%s", averages[i], i))
end
local total = 0
for _, avg in pairs(averages) do total = total + avg end
print(string.format("%7.3f\t%s", total, "total"))
