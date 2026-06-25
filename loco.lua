-- loco.lua
-- Electric Locomotive Control Program
-- Peripherals:
--   All I/O sides are configured in config_swd1p.lua (sideMonitor, sideSetSpeed, etc.)

-- ============================================================
-- 0. Load config
-- ============================================================
local cfg = dofile("config_swd1p.lua")

local MAX_SPEED      = cfg.maxSpeed
local SIDE_SETSPEED  = cfg.sideSetSpeed
local SIDE_REVERSE   = cfg.sideReverse
local SIDE_MONITOR   = cfg.sideMonitor
local SIDE_BRAKE     = cfg.sideBrake
local SIDE_THROTTLE  = cfg.sideThrottle
local SIDE_REVGEARBOX = cfg.sideRevGearbox

-- ============================================================
-- 1. Redstone I/O helpers
-- ============================================================

-- Read set speed signal, convert to m/s
local function getSetSpeed()
    local sig = redstone.getAnalogInput(SIDE_SETSPEED)
    return (sig / 15) * MAX_SPEED
end

-- Read reverse signal; returns true if locomotive should run in reverse
local function isReverse()
    local sig = redstone.getInput(SIDE_REVERSE)
    if REVERSE_INVERTED then
        return not sig   -- inverted: no signal = reverse
    else
        return sig       -- normal:   signal    = reverse
    end
end

-- Set brake output (analog, 0=off, 15=full brake)
-- Use ceil so any non-zero brake demand always produces at least signal 1
local function setBrake(level)
    local out = (level > 0) and math.ceil(math.max(0, math.min(15, level))) or 0
    redstone.setAnalogOutput(SIDE_BRAKE, out)
end

-- Set reverse gearbox (digital HIGH = activate)
local function setReverseGearbox(activate)
    redstone.setOutput(SIDE_REVGEARBOX, activate)
end

-- Set throttle output to gearbox (inverted: 0->15, 15->0)
local function setThrottleOutput(throttleInput)
    local output = 15 - throttleInput
    redstone.setAnalogOutput(SIDE_THROTTLE, output)
end

-- ============================================================
-- Unified PID controller (throttle + brake in one)
-- ============================================================
local PID_DT       = cfg.pidDt
local PID_KI       = cfg.pidKi
local PID_KD       = cfg.pidKd
local KP_NEAR      = cfg.pidKpNear
local KP_MID       = cfg.pidKpMid
local KP_FAR       = cfg.pidKpFar
local THRESH_NEAR  = cfg.nearThreshold
local THRESH_FAR   = cfg.farThreshold
local DEADBAND     = cfg.pidDeadband
local BRAKE_SCALE  = cfg.brakeScale
local EMERG_KP     = cfg.emergencyBrakeKp

local pid = { integral = 0, lastError = 0 }

local function pidReset()
    pid.integral  = 0
    pid.lastError = 0
end

-- Variable gain: returns Kp based on absolute error magnitude
local function gainSchedule(absErr)
    if absErr < THRESH_NEAR then
        return KP_NEAR
    elseif absErr < THRESH_FAR then
        -- Linear interpolation between KP_NEAR and KP_MID / KP_MID and KP_FAR
        local t = (absErr - THRESH_NEAR) / (THRESH_FAR - THRESH_NEAR)
        return KP_MID + t * (KP_FAR - KP_MID)
    else
        return KP_FAR
    end
end

-- Unified PID compute:
--   returns driveOut [0,15] and brakeOut [0,15]
--   positive PID output -> throttle; negative -> brake
--   setSpeed==0: emergency brake, bypass PID
local function pidCompute(setSpeed, actualSpeed)
    -- Emergency stop
    if setSpeed <= 0 then
        pidReset()
        if actualSpeed > 0.02 then
            return 0, math.min(15, EMERG_KP * actualSpeed * BRAKE_SCALE)
        else
            return 0, 0
        end
    end

    local err = setSpeed - actualSpeed
    local absErr = math.abs(err)

    -- Deadband: freeze integral near setpoint
    -- If still moving faster than setpoint inside deadband, keep minimum brake
    if absErr < DEADBAND then
        pid.lastError = err
        if err < 0 and actualSpeed > 0.02 then
            return 0, 1  -- minimum brake signal to finish stopping (not scaled)
        end
        return 0, 0
    end

    -- Variable Kp
    local Kp = gainSchedule(absErr)

    -- Integral (anti-windup ±30)
    pid.integral = math.max(-30, math.min(30, pid.integral + err * PID_DT))

    -- Derivative
    local D = PID_KD * (err - pid.lastError) / PID_DT
    pid.lastError = err

    local output = Kp * err + PID_KI * pid.integral + D

    -- Split into throttle / brake
    -- brakeScale is applied only to brake side; throttle is unaffected
    if output >= 0 then
        return math.min(15, output), 0
    else
        return 0, math.min(15, -output * BRAKE_SCALE)
    end
end


-- ============================================================
-- 2. Velocity sensor (auto-scan all sides)
-- ============================================================
local sensor = nil
for _, side in ipairs({"top","bottom","front","back","right","left"}) do
    if peripheral.getType(side) == "velocity_sensor" then
        sensor = peripheral.wrap(side)
        print("Velocity sensor found on: " .. side)
        break
    end
end
if not sensor then
    print("WARNING: No velocity_sensor found. Actual speed will show N/A.")
end

-- Read actual speed from sensor; returns nil if unavailable
local function getActualSpeed()
    if not sensor then return nil end
    local ok, v = pcall(sensor.getVelocity)
    if ok and type(v) == "number" then return math.abs(v) end
    return nil
end

-- ============================================================
-- 3. Monitor helpers
-- ============================================================
local monitor = peripheral.wrap(SIDE_MONITOR)
if not monitor then
    print("ERROR: No monitor found on side: " .. SIDE_MONITOR)
    return
end

local function mCall(method, ...)
    if monitor[method] then
        pcall(monitor[method], ...)
    end
end

-- Shrink text so more info fits on a 1x1 monitor
mCall("setTextScale", 0.5)
mCall("setBackgroundColor", colors.black)
mCall("clear")

-- ============================================================
-- 4. Read computer name and determine cab end
-- ============================================================
local locoName = os.getComputerLabel() or ("CC#" .. os.getComputerID())

-- Determine reverse signal polarity from cab suffix:
--   name ending in "A" -> A-end cab, reverseSignalInverted = false (HIGH = reverse)
--   name ending in "B" -> B-end cab, reverseSignalInverted = true  (LOW  = reverse)
--   unknown suffix     -> default false, print a warning
local cabEnd = string.upper(string.sub(locoName, -1))
local REVERSE_INVERTED
if cabEnd == "A" then
    REVERSE_INVERTED = false
elseif cabEnd == "B" then
    REVERSE_INVERTED = true
else
    REVERSE_INVERTED = false
    print("WARNING: cab end unknown ('" .. cabEnd .. "'), defaulting to A-end mode")
end
print("Cab end: " .. cabEnd .. " | REVERSE_INVERTED=" .. tostring(REVERSE_INVERTED))

-- ============================================================
-- 5. Display update
-- ============================================================
-- Speed color: green=slow, yellow=mid, red=fast, gray=stopped
local function speedColor(v)
    if v <= 0 then
        return colors.lightGray
    elseif v < MAX_SPEED * 0.4 then
        return colors.green
    elseif v < MAX_SPEED * 0.8 then
        return colors.yellow
    else
        return colors.red
    end
end

-- updateDisplay(setSpeed, actualSpeed, driveOut, brakeOut, reverse)
local function updateDisplay(setSpeed, actualSpeed, driveOut, brakeOut, reverse)
    mCall("clear")

    -- Line 1: Locomotive name
    mCall("setCursorPos", 1, 1)
    mCall("setTextColor", colors.yellow)
    monitor.write(locoName)

    -- Line 2: Set speed (from top redstone signal)
    mCall("setCursorPos", 1, 2)
    mCall("setTextColor", speedColor(setSpeed))
    monitor.write(string.format("Set:%.1fm/s", setSpeed))

    -- Line 3: Actual speed from sensor (0 if sensor unavailable)
    mCall("setCursorPos", 1, 3)
    mCall("setTextColor", speedColor(actualSpeed))
    monitor.write(string.format("Spd:%.1fm/s ", actualSpeed))

    -- Line 4: Drive throttle output bar
    mCall("setCursorPos", 1, 4)
    mCall("setTextColor", colors.cyan)
    local dBar = string.rep("|", math.floor((driveOut/15)*8)) .. string.rep(".", 8 - math.floor((driveOut/15)*8))
    monitor.write(string.format("T:%s%2d", dBar, math.floor(driveOut)))

    -- Line 5: Brake output bar
    mCall("setCursorPos", 1, 5)
    if brakeOut > 0 then
        mCall("setTextColor", colors.red)
    else
        mCall("setTextColor", colors.gray)
    end
    local bBar = string.rep("|", math.floor((brakeOut/15)*8)) .. string.rep(".", 8 - math.floor((brakeOut/15)*8))
    monitor.write(string.format("B:%s%2d", bBar, math.floor(brakeOut)))

    -- Line 6: Direction
    mCall("setCursorPos", 1, 6)
    if reverse then
        mCall("setTextColor", colors.orange)
        monitor.write("DIR: << REV")
    else
        mCall("setTextColor", colors.lime)
        monitor.write("DIR:  FWD >>")
    end

    -- Line 7: Time
    mCall("setCursorPos", 1, 7)
    mCall("setTextColor", colors.lightGray)
    monitor.write(os.date("%H:%M:%S") .. (brakeOut > 0 and " BRK" or ""))
end

-- ============================================================
-- 6. Main loop
-- ============================================================
print("Locomotive program started: " .. locoName)
print("Max speed: " .. MAX_SPEED .. " m/s | Ctrl+T to stop")

-- Ensure all outputs are off at startup
setBrake(0)
setThrottleOutput(0)
redstone.setOutput("left", false)

local prevSetSpeed = -1

while true do
    local setSpeed    = getSetSpeed()          -- desired speed magnitude (m/s, always >= 0)
    local reverse     = isReverse()            -- true = reverse relative to A-end
    local actualSpeed = getActualSpeed() or 0

    -- Determine if reverse gearbox should be activated
    -- A-end: activate on reverse; B-end: activate on forward (XOR with REVERSE_INVERTED)
    local needReverse = (reverse and not REVERSE_INVERTED) or (not reverse and REVERSE_INVERTED)
    setReverseGearbox(reverse)

    -- PID always works with positive speed magnitude
    -- Reset PID integral on large setpoint step changes
    if math.abs(setSpeed - prevSetSpeed) > 1.0 then
        pidReset()
    end
    prevSetSpeed = setSpeed

    -- Unified PID: single output splits into throttle and brake
    local driveOut, brakeOut = pidCompute(setSpeed, actualSpeed)

    setThrottleOutput(driveOut)
    setBrake(brakeOut)

    updateDisplay(setSpeed, actualSpeed, driveOut, brakeOut, needReverse)

    sleep(PID_DT)
end
