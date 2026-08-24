#!/usr/bin/env python3

# Copyright 2026 Project19 contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

from PIL import Image, ImageChops, ImageStat


MIN_DURATION_S = 30.0
MAX_DURATION_S = 180.0
MIN_WIDTH = 1280
MIN_HEIGHT = 720
MIN_HALF_LUMA = 8.0
MIN_HALF_STDDEV = 8.0
MIN_MOTION_DELTA = 0.35


def parse_args():
    parser = argparse.ArgumentParser(
        description="Validate the complete side-by-side M4 demonstration video."
    )
    parser.add_argument("video", type=Path)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--contact-sheet", type=Path, required=True)
    parser.add_argument("--work-dir", type=Path, required=True)
    return parser.parse_args()


def run_json(command):
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads(result.stdout)


def probe_video(video):
    if not video.is_file() or video.stat().st_size == 0:
        raise RuntimeError(f"video is missing or empty: {video}")
    probe = run_json(
        [
            "ffprobe",
            "-v",
            "error",
            "-show_streams",
            "-show_format",
            "-of",
            "json",
            str(video),
        ]
    )
    video_streams = [
        stream for stream in probe.get("streams", [])
        if stream.get("codec_type") == "video"
    ]
    if len(video_streams) != 1:
        raise RuntimeError("video must contain exactly one video stream")
    stream = video_streams[0]
    width = int(stream.get("width", 0))
    height = int(stream.get("height", 0))
    duration = float(probe.get("format", {}).get("duration", "nan"))
    if not math.isfinite(duration) or not MIN_DURATION_S <= duration <= MAX_DURATION_S:
        raise RuntimeError(
            f"video duration {duration!r} is outside "
            f"[{MIN_DURATION_S}, {MAX_DURATION_S}] seconds"
        )
    if width < MIN_WIDTH or height < MIN_HEIGHT:
        raise RuntimeError(
            f"video resolution {width}x{height} is below {MIN_WIDTH}x{MIN_HEIGHT}"
        )
    if width % 2:
        raise RuntimeError("side-by-side video width must be even")
    return {"duration_s": duration, "width": width, "height": height}


def extract_frames(video, duration, directory):
    sample_times = [duration * fraction for fraction in (0.15, 0.5, 0.85)]
    frames = []
    for index, sample_time in enumerate(sample_times):
        output = directory / f"frame_{index}.png"
        subprocess.run(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-ss",
                f"{sample_time:.6f}",
                "-i",
                str(video),
                "-frames:v",
                "1",
                "-y",
                str(output),
            ],
            check=True,
        )
        if not output.is_file() or output.stat().st_size == 0:
            raise RuntimeError(f"failed to extract video frame at {sample_time:.3f}s")
        frames.append(Image.open(output).convert("RGB"))
    return sample_times, frames


def half_statistics(image):
    grayscale = image.convert("L")
    stats = ImageStat.Stat(grayscale)
    return {"mean_luma": stats.mean[0], "stddev_luma": stats.stddev[0]}


def mean_difference(first, second):
    difference = ImageChops.difference(first.convert("RGB"), second.convert("RGB"))
    return sum(ImageStat.Stat(difference).mean) / 3.0


def analyze_frames(frames):
    if len(frames) < 3:
        raise RuntimeError("at least three sampled frames are required")
    left_frames = []
    right_frames = []
    samples = []
    for frame in frames:
        width, height = frame.size
        left = frame.crop((0, 0, width // 2, height))
        right = frame.crop((width // 2, 0, width, height))
        left_stats = half_statistics(left)
        right_stats = half_statistics(right)
        for name, stats in (("left", left_stats), ("right", right_stats)):
            if stats["mean_luma"] < MIN_HALF_LUMA:
                raise RuntimeError(f"{name} video half is effectively black")
            if stats["stddev_luma"] < MIN_HALF_STDDEV:
                raise RuntimeError(f"{name} video half lacks visible scene detail")
        left_frames.append(left)
        right_frames.append(right)
        samples.append({"left": left_stats, "right": right_stats})

    left_deltas = [
        mean_difference(left_frames[index - 1], left_frames[index])
        for index in range(1, len(left_frames))
    ]
    right_deltas = [
        mean_difference(right_frames[index - 1], right_frames[index])
        for index in range(1, len(right_frames))
    ]
    if max(left_deltas) < MIN_MOTION_DELTA:
        raise RuntimeError("left video half is static across sampled mission times")
    if max(right_deltas) < MIN_MOTION_DELTA:
        raise RuntimeError("right video half is static across sampled mission times")
    return {
        "samples": samples,
        "left_motion_deltas": left_deltas,
        "right_motion_deltas": right_deltas,
    }


def write_contact_sheet(frames, output):
    cell_width = 960
    cell_height = 540
    resampling = getattr(Image, "Resampling", Image)
    sheet = Image.new("RGB", (cell_width * 2, cell_height * 2), "black")
    for index, frame in enumerate(frames):
        thumbnail = frame.copy()
        thumbnail.thumbnail((cell_width, cell_height), resampling.LANCZOS)
        x = (index % 2) * cell_width + (cell_width - thumbnail.width) // 2
        y = (index // 2) * cell_height + (cell_height - thumbnail.height) // 2
        sheet.paste(thumbnail, (x, y))
    output.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(output)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json_atomic(data, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}-", dir=output.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as file:
            json.dump(data, file, indent=2, sort_keys=True)
            file.write("\n")
        os.replace(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)


def main():
    args = parse_args()
    probe = probe_video(args.video)
    args.work_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=args.work_dir) as temporary_directory:
        sample_times, frames = extract_frames(
            args.video, probe["duration_s"], Path(temporary_directory)
        )
        visual = analyze_frames(frames)
        write_contact_sheet(frames, args.contact_sheet)
    metrics = {
        "accepted": True,
        "video": {
            **probe,
            "path": str(args.video.resolve()),
            "sha256": sha256_file(args.video),
            "sample_times_s": sample_times,
        },
        "visual": visual,
    }
    write_json_atomic(metrics, args.metrics)
    print(
        f"M4 video accepted: {probe['duration_s']:.3f}s, "
        f"{probe['width']}x{probe['height']}, both halves visible and dynamic"
    )
    print(f"wrote {args.metrics}")


if __name__ == "__main__":
    main()
