import ctypes
import platform
import os
import sys
import matplotlib.pyplot as plt
import numpy as np


def load_library():
    build_dir = os.path.join(os.path.dirname(__file__), "..", "build")
    if platform.system() == "Windows":
        candidates = ["acc.dll", "libacc.dll"]
    else:
        candidates = ["libacc.so", "libacc.dylib"]

    for name in candidates:
        path = os.path.join(build_dir, name)
        if os.path.exists(path):
            return ctypes.CDLL(path)

    print(f"ERROR: ACC shared library not found in {build_dir}")
    print("Run: cmake -B build && cmake --build build")
    sys.exit(1)


lib = load_library()


lib.acc_create.restype = ctypes.c_void_p
lib.acc_destroy.argtypes = [ctypes.c_void_p]
lib.acc_reset.argtypes = [ctypes.c_void_p]
lib.acc_update.argtypes = [
    ctypes.c_void_p,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_int,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
]
lib.acc_get_state.argtypes = [ctypes.c_void_p]
lib.acc_get_state.restype = ctypes.c_int

STATE_NAMES = {0: "FREE_CRUISE", 1: "FOLLOWING", 2: "EMERGENCY"}
CYCLE_TIME = 0.01


class AccController:
    def __init__(self):
        self._h = lib.acc_create()
        self._throttle = ctypes.c_float(0.0)
        self._brake = ctypes.c_float(0.0)

    def __del__(self):
        lib.acc_destroy(self._h)

    def reset(self):
        lib.acc_reset(self._h)

    def step(
        self,
        ego_speed: float,
        target_distance: float,
        set_speed: float,
        vehicle_detected: bool,
    ):
        lib.acc_update(
            self._h,
            ctypes.c_float(ego_speed),
            ctypes.c_float(target_distance),
            ctypes.c_float(set_speed),
            ctypes.c_int(1 if vehicle_detected else 0),
            ctypes.byref(self._throttle),
            ctypes.byref(self._brake),
        )
        return (
            self._throttle.value,
            self._brake.value,
            STATE_NAMES[lib.acc_get_state(self._h)],
        )


def apply_dynamics(ego_speed: float, throttle: float, brake: float) -> float:
    accel = throttle * 3.0 - brake * 6.0
    return max(0.0, ego_speed + accel * CYCLE_TIME)


def scenario_open_road(t: float, ego_speed: float):
    return ego_speed, 999.0, 30.0, False


def scenario_following(t: float, ego_speed: float):
    if t < 5.0:
        return ego_speed, 999.0, 30.0, False
    lead_speed = max(8.0, 30.0 - (t - 5.0) * 1.5)
    distance = max(5.0, 40.0 - max(0.0, ego_speed - lead_speed) * (t - 5.0))
    return ego_speed, distance, 30.0, True


def scenario_emergency(t: float, ego_speed: float):
    if t < 8.0:
        return ego_speed, 999.0, 30.0, False
    return ego_speed, 6.0, 30.0, True


def scenario_resume(t: float, ego_speed: float):
    if t < 3.0:
        return ego_speed, 999.0, 30.0, False
    if t < 12.0:
        return ego_speed, 35.0, 30.0, True
    return ego_speed, 999.0, 30.0, False


def run_scenario(name: str, fn, duration: float = 25.0) -> dict:
    ctrl = AccController()
    t_vec = np.arange(0.0, duration, CYCLE_TIME)
    ego_speed = 0.0

    log = {
        k: [] for k in ("time", "ego_speed", "distance", "throttle", "brake", "state")
    }

    for t in t_vec:
        ego_speed, distance, set_speed, detected = fn(t, ego_speed)
        throttle, brake, state = ctrl.step(ego_speed, distance, set_speed, detected)
        ego_speed = apply_dynamics(ego_speed, throttle, brake)

        log["time"].append(t)
        log["ego_speed"].append(ego_speed)
        log["distance"].append(distance)
        log["throttle"].append(throttle)
        log["brake"].append(brake)
        log["state"].append(state)

    return log


def validate(name: str, checks: list) -> bool:
    ok = all(checks)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    return ok


def plot(logs: dict):
    fig, axes = plt.subplots(len(logs), 3, figsize=(15, 4 * len(logs)))
    fig.suptitle("ACC SIL — Scenario Validation", fontsize=13, y=1.01)

    state_to_int = {"FREE_CRUISE": 0, "FOLLOWING": 1, "EMERGENCY": 2}

    for row, (name, r) in enumerate(logs.items()):
        t = r["time"]
        ax = axes[row] if len(logs) > 1 else axes

        ax[0].plot(t, r["ego_speed"], label="ego speed (m/s)", color="steelblue")
        ax[0].set_title(name)
        ax[0].set_ylabel("Speed (m/s)")
        ax[0].legend(fontsize=8)

        ax[1].plot(t, r["throttle"], label="throttle", color="green")
        ax[1].plot(t, r["brake"], label="brake", color="red")
        ax[1].set_ylabel("Command")
        ax[1].legend(fontsize=8)

        s_int = [state_to_int[s] for s in r["state"]]
        ax[2].step(t, s_int, where="post", color="purple")
        ax[2].set_yticks([0, 1, 2])
        ax[2].set_yticklabels(["FREE", "FOLLOW", "EMERG"], fontsize=8)
        ax[2].set_ylabel("State")
        ax[2].set_xlabel("Time (s)")

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "sil_results.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\nPlot saved → {out_path}")
    plt.show()


if __name__ == "__main__":
    print("ACC Software-in-the-Loop Harness")
    print("=" * 40)

    logs = {}
    all_pass = True

    print("\nScenario 1: Open Road")
    r1 = run_scenario("Open Road", scenario_open_road, 30.0)
    logs["1. Open Road"] = r1
    all_pass &= validate("Reaches set speed (>27 m/s)", [r1["ego_speed"][-1] > 27.0])
    all_pass &= validate(
        "Stays in FREE_CRUISE", [all(s == "FREE_CRUISE" for s in r1["state"])]
    )

    print("\nScenario 2: Following Lead Vehicle")
    r2 = run_scenario("Following", scenario_following, 25.0)
    logs["2. Following"] = r2
    all_pass &= validate("Transitions to FOLLOWING", ["FOLLOWING" in r2["state"]])
    all_pass &= validate("Brake applied when too close", [max(r2["brake"]) > 0.0])

    print("\nScenario 3: Emergency Cut-In")
    r3 = run_scenario("Emergency Cut-In", scenario_emergency, 20.0)
    logs["3. Emergency"] = r3
    all_pass &= validate("Enters EMERGENCY state", ["EMERGENCY" in r3["state"]])
    all_pass &= validate(
        "Full brake (brake_cmd == 1.0) applied", [max(r3["brake"]) == 1.0]
    )
    all_pass &= validate(
        "Zero throttle during emergency",
        [
            all(
                t == 0.0
                for t, s in zip(r3["throttle"], r3["state"])
                if s == "EMERGENCY"
            )
        ],
    )

    print("\nScenario 4: Lead Vehicle Clears Lane")
    r4 = run_scenario("Resume", scenario_resume, 20.0)
    logs["4. Resume"] = r4
    all_pass &= validate(
        "Returns to FREE_CRUISE after clearance", [r4["state"][-1] == "FREE_CRUISE"]
    )

    print("\n" + "=" * 40)
    print(f"Result: {'ALL SCENARIOS PASS' if all_pass else 'FAILURES DETECTED'}")

    plot(logs)
