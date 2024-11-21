#!/usr/bin/env python3
"""
Demonstrates how to log simple plots with the Rerun SDK.

Run:
```sh
./examples/python/plot/plots.py
```
"""

from __future__ import annotations

import argparse
import random
from math import cos, sin, tau

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import control as ct

DESCRIPTION = """
Desciption goes here.
""".strip()


def clamp(n, smallest, largest):  # type: ignore[no-untyped-def]
    return max(smallest, min(n, largest))


def log_quadcopter_simulation(result) -> None:
    # Group the outputs into meaningful categories

    t = result.t

    groups = {
        "position": ["pos_x", "pos_y", "pos_z"],
        "velocity": ["vel_x", "vel_y", "vel_z"],
        "orientation": ["phi", "theta", "psi"],
        "angular_rates": ["p", "q", "r"],
        "rotor_commands": ["r1", "r2", "r3", "r4"],
        "forces_torques": ["thrust", "torque_x", "torque_y", "torque_z"]
    }
    
    colors = {
        "x": [255, 0, 0],    # red
        "y": [0, 255, 0],    # green
        "z": [0, 0, 255],    # blue
        "phi": [255, 0, 0],
        "theta": [0, 255, 0],
        "psi": [0, 0, 255],
        "p": [255, 0, 0],
        "q": [0, 255, 0],
        "r": [0, 0, 255],
        "r1": [255, 0, 0],
        "r2": [0, 255, 0],
        "r3": [0, 255, 255],
        "r4": [255, 0, 255],
        "thrust": [255, 255, 0],
        "torque_x": [255, 0, 0],
        "torque_y": [0, 255, 0],
        "torque_z": [0, 0, 255],
    }

    for frame, time in enumerate(t):
        rr.set_time_sequence("frame_nr", frame)
        
        # Log each group of measurements
        for group_name, signals in groups.items():
            for idx, signal in enumerate(signals):
                signal_idx = result.output_labels.index(signal)
                value = result.outputs[signal_idx][frame]

                rr.log(
                    f"quadcopter/{group_name}/{signal}",
                    rr.Scalar(value),
                    rr.SeriesLine(
                        color=colors.get(signal.split('_')[-1], colors.get(signal, [255, 255, 255])),
                        name=signal
                    )
                )


def plot_main(result) -> None:
    parser = argparse.ArgumentParser(
        description="demonstrates how to integrate python's native `logging` with the Rerun SDK"
    )
    rr.init("quadcopter_simulation", spawn=True)
    rr.script_add_args(parser)
    args = parser.parse_args()

    args.save = "sim.rrd"

    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Grid(
                rrb.TimeSeriesView(
                    name="Position",
                    origin="/quadcopter/position",
                ),
                rrb.TimeSeriesView(
                    name="Velocity",
                    origin="/quadcopter/velocity",
                ),
                rrb.TimeSeriesView(
                    name="Orientation",
                    origin="/quadcopter/orientation",
                ),
                rrb.TimeSeriesView(
                    name="Angular Rates",
                    origin="/quadcopter/angular_rates",
                ),
                rrb.TimeSeriesView(
                    name="Rotor Commands",
                    origin="/quadcopter/rotor_commands",
                ),
                rrb.TimeSeriesView(
                    name="Forces & Torques",
                    origin="/quadcopter/forces_torques",
                ),
            ),
            rrb.TextDocumentView(name="Description", origin="/description"),
            column_shares=[3, 1],
        ),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
    )

    rr.script_setup(args, "rerun_example_plot", default_blueprint=blueprint)

    rr.log("description", rr.TextDocument(DESCRIPTION, media_type=rr.MediaType.MARKDOWN), static=True)
    
    # Log the simulation results
    log_quadcopter_simulation(result)

    rr.script_teardown(args)