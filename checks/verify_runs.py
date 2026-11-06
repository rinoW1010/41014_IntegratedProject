import json
from pathlib import Path
import numpy as np

for name in ("nominal", "varied", "loss"):
    folder = Path("outputs") / f"final_{name}"
    metrics = json.loads((folder / "metrics.json").read_text())
    rows = np.genfromtxt(folder / "trial.csv", delimiter=",", names=True)
    assert not metrics["collision"]
    assert not metrics["false_charging"]
    assert metrics["driven_path_clear_with_margin"]

    if name == "loss":
        assert not metrics["success"] and not metrics["charging"]
        lost = np.flatnonzero(
            (rows["docking"] == 1) & (rows["aligning"] == 0) & (rows["accepted"] == 0)
        )
        assert len(lost) > 1
        # Camera rejection is recorded after this steps command
        # The following decision must command 0 motion
        stopped = rows[lost[0] + 1:]
        assert np.allclose(stopped["v"], 0)
        assert np.allclose(stopped["w"], 0)
    else:
        assert metrics["success"] and metrics["charging"]
        holding = rows[rows["held"] > 0]
        assert len(holding) >= 20
        assert np.all(holding["accepted"] == 1)
        assert np.allclose(holding["v"], 0)
        assert np.allclose(holding["w"], 0)
        assert rows["held"][-1] >= 1
    print(name, "checks passed")
        
