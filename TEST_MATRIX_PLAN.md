# Input Mode Test Matrix — C-First Vectors

## Existing vectors (already passing)
- `bidir_dshot.txt` — BidirDshot normal (forward/reverse/zero)
- `rc_car_reverse.txt` — BidirDshotRcCar brake/return-to-center/flip
- `servo_bidir.txt` — BidirServo direction flip + dead band
- `servo_bidir_speed_gate.txt` — BidirServo reversal blocked at high RPM
- `servo_rc_car_bidir.txt` — BidirServoRcCar brake/center/flip
- `stuck_rotor.txt` — Unidirectional fault latch
- `brake_modes.txt` — Unidirectional brake_on_stop (partial)
- `bemf_timeout.txt` — Unidirectional fault latch (pre-set counter)

## Vectors to create (C-first, golden values from C harness)

### 1. Unidirectional
- [x] `uni_brake_on_stop.txt` — brake_on_stop mode 1 with comp_pwm: verify pwm_duty when stopped

### 2. BidirDshot
- [x] `bidir_dshot_fault.txt` — stuck rotor during bidir DShot operation
- [x] `bidir_dshot_recovery.txt` — fault clear by centering stick (input=0)

### 3. BidirDshotRcCar
- [x] `bidir_dshot_rccar_fault.txt` — stuck rotor during RC-car operation

### 4. BidirServo
- [x] `servo_bidir_fault.txt` — stuck rotor during servo bidir

### 5. BidirServoRcCar
- [x] `servo_rccar_fault.txt` — stuck rotor during servo RC-car bidir

## Method
1. Write C harness command sequence
2. Capture golden output
3. Write .txt vector with assertions from golden output
4. Verify vector passes on C harness
5. Verify vector passes on Rust harness
6. If Rust fails, that's a bug to fix in the InputMode enum refactor
