"""
Run robot navigation and visual docking demonstration
"""

import argparse
from pathlib import Path

from turtlebot_project.mission import mission

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--varied", action = "store_true", help = "Start with a six-degree heading variation")
    parser.add_argument("--output", type = Path, help = "Folder for trial results")
    parser.add_argument("--frames", action = "store_true", help = "Save overview images for demo video")
    parser.add_argument("--marker-loss", action = "store_true", help = "Test stopping when camera corrections are lost during docking")
    parser.add_argument("--heading-gain", type = float, default = 1.8, help = "Heading feedback gain used during route tracking")
    args = parser.parse_args()

    mission(
        varied = args.varied,
        output = args.output,
        frames = args.frames,
        marker_loss = args.marker_loss,
        heading_gain = args.heading_gain
        )

if __name__ == "__main__":
    main()