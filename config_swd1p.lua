-- config_swd1p.lua
-- Locomotive configuration file for SWD1P
-- Edit this file to change locomotive parameters

return {
    -- Maximum operating speed (m/s)
    -- Maps redstone signal 15 -> this speed, signal 0 -> 0 (stop)
    maxSpeed = 25.0,

    -- ---- Redstone I/O sides ----
    -- INPUT sides
    sideSetSpeed  = "top",    -- analog INPUT:  speed setpoint (0-15)
    sideReverse   = "right",  -- digital INPUT: reverse signal

    -- OUTPUT sides
    sideMonitor   = "left",   -- monitor peripheral side
    sideBrake     = "bottom",  -- analog OUTPUT: brake (0=off, 15=full)
    sideThrottle  = "back",   -- analog OUTPUT: throttle to gearbox (inverted)
    sideRevGearbox = "left", -- digital OUTPUT: reverse gearbox activate

    -- ---- RHI notch speed step (loco_rhi.lua only) ----
    -- Speed change per loop tick per notch-strength unit (m/s).
    -- notch 1 (max decel) applies 6x this value, notch 6 (min decel) applies 1x.
    -- notch 9 (min accel) applies 1x this value, notch 15 (max accel) applies 7x.
    -- Lower -> finer speed control, slower response. Higher -> coarser but faster.
    rhiStep = 0.2,

    -- ---- PID loop interval ----
    -- How often the control loop runs (seconds). Smaller = more responsive but more CPU.
    -- Must stay consistent with the actual sleep() call in loco.lua.
    pidDt = 0.1,

    -- ---- Unified PID (single controller for both throttle and brake) ----
    -- Positive output [0,15]  -> throttle (drive)
    -- Negative output [-15,0] -> brake    (decelerate)
    -- This prevents throttle/brake fighting and eliminates oscillation at setpoint.

    -- Ki (Integral): Eliminates steady-state speed error over time.
    --   Higher -> faster correction of persistent error (e.g. uphill load).
    --   Lower  -> may never fully reach setpoint under heavy load.
    pidKi = 0.6,

    -- Kd (Derivative): Dampens oscillation by reacting to speed change rate.
    --   Higher -> smoother approach, but amplifies sensor noise.
    --   Lower  -> may overshoot setpoint slightly.
    pidKd = 0.4,

    -- ---- Variable gain (gain scheduling) ----
    -- Kp is scaled by error magnitude in three zones for smooth yet powerful control.
    -- Zone boundaries are speed errors in m/s.
    --
    -- Near zone  (|error| < nearThreshold): gentle Kp, avoids overshoot near setpoint
    -- Mid zone   (nearThreshold <= |error| < farThreshold): normal Kp
    -- Far zone   (|error| >= farThreshold): high Kp, fast response to large deviations
    pidKpNear = 1.0,    -- Kp when error is small  (smooth, no jitter)
    pidKpMid  = 1.5,    -- Kp when error is medium (normal response)
    pidKpFar  = 3.0,    -- Kp when error is large  (aggressive catch-up)
    nearThreshold = 0.8, -- m/s: boundary between near and mid zones
    farThreshold  = 5.0, -- m/s: boundary between mid and far zones

    -- ---- Deadband ----
    -- If |error| < deadband, integral is frozen and output is zero.
    -- Prevents micro-corrections and throttle/brake toggling at setpoint.
    -- Set to 0 to disable.
    pidDeadband = 0.1,  -- m/s

    -- ---- Brake output scale ----
    -- Multiplier applied to all brake outputs (both PID and emergency), range (0, 1].
    -- Does NOT affect PID error calculation or integral - only scales the final signal.
    -- Lower -> gentler braking, longer stopping distance.
    -- 1.0   -> full PID output sent to brake (default).
    brakeScale = 0.2,

    -- ---- Emergency brake ----
    -- When setSpeed == 0, apply brake proportional to actual speed * this gain.
    -- Independent of PID; acts immediately without integral windup.
    emergencyBrakeKp = 1.5,
}
