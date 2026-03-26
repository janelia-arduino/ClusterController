#!/usr/bin/env python3
"""Sanity-check that ClusterController.qm contains the homing failure handling
relied on by the current generated sources.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
QM_PATH = REPO_ROOT / "src" / "ClusterController.qm"


def _find_class(root: ET.Element, name: str) -> ET.Element:
    for cls in root.iter("class"):
        if cls.attrib.get("name") == name:
            return cls
    raise RuntimeError(f"class {name!r} not found")


def _find_state(root_state: ET.Element, name: str) -> ET.Element | None:
    if root_state.attrib.get("name") == name:
        return root_state
    for child in root_state.findall("state"):
        found = _find_state(child, name)
        if found is not None:
            return found
    return None


def _require(condition: bool, msg: str, errors: list[str]) -> None:
    if not condition:
        errors.append(msg)


def main() -> int:
    root = ET.parse(QM_PATH).getroot()
    errors: list[str] = []

    prism = _find_class(root, "Prism")
    prism_attrs = {a.attrib.get("name") for a in prism.findall("attribute")}
    _require("home_status_" in prism_attrs,
             "Prism is missing home_status_ in ClusterController.qm",
             errors)
    sc = prism.find("statechart")
    _require(sc is not None, "Prism statechart missing in ClusterController.qm", errors)

    if sc is not None:
        homing = None
        enabled = None
        for top in sc.findall("state"):
            if homing is None:
                homing = _find_state(top, "Homing")
            if enabled is None:
                enabled = _find_state(top, "Enabled")

        _require(homing is not None, "Prism::Homing state missing in ClusterController.qm", errors)
        _require(enabled is not None, "Prism::Enabled state missing in ClusterController.qm", errors)

        if homing is not None:
            timeout = None
            for tran in homing.findall("tran"):
                if tran.attrib.get("trig") == "CLUSTER_TIMEOUT":
                    timeout = tran
                    break

            _require(timeout is not None, "Prism::Homing is missing CLUSTER_TIMEOUT handling", errors)
            if timeout is not None:
                tran_action = timeout.findtext("action") or ""
                _require("Prism_checkHome" in tran_action,
                         "Prism::Homing CLUSTER_TIMEOUT does not call Prism_checkHome before branching",
                         errors)

                top_choices = timeout.findall("choice")
                _require(len(top_choices) >= 3,
                         "Prism::Homing CLUSTER_TIMEOUT should model homeSucceeded/homeFailed/else choices",
                         errors)
                if len(top_choices) >= 3:
                    first_guard = top_choices[0].findtext("guard") or ""
                    second_guard = top_choices[1].findtext("guard") or ""
                    third_guard = top_choices[2].findtext("guard") or ""

                    _require("Prism_homeSucceeded" in first_guard,
                             "Prism::Homing first choice does not test Prism_homeSucceeded",
                             errors)
                    _require("Prism_homeFailed" in second_guard,
                             "Prism::Homing second choice does not test Prism_homeFailed",
                             errors)
                    _require(third_guard.strip() == "",
                             "Prism::Homing third choice should be the else branch",
                             errors)

                    second_target = top_choices[1].attrib.get("target", "")
                    _require(second_target == "../../1",
                             "Prism::Homing homeFailed branch should transition back to Enabled",
                             errors)

    if errors:
        for msg in errors:
            print(f"ERROR: {msg}")
        return 1

    print("ClusterController.qm contains the expected homing failure recovery changes.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
