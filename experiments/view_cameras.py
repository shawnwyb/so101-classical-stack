"""Desktop camera previews with optional per-camera AVI recording; no arm control."""

import argparse
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import signal
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
from PIL import Image, ImageTk

from experiments.camera_settings import DEFAULT_CONFIG, apply_controls, inspect_controls, load_config


def discover_cameras():
    """Prefer stable USB paths, avoiding the usual secondary metadata nodes."""
    devices = sorted(Path('/dev/v4l/by-id').glob('*video-index0'))
    if not devices:
        devices = sorted(Path('/dev').glob('video[0-9]*'))
    return [(device.name, str(device)) for device in devices]


def camera_argument(value):
    name, separator, source = value.partition('=')
    if not separator:
        source = name
    if not name or not source:
        raise argparse.ArgumentTypeError('Use --camera NAME=SOURCE or --camera SOURCE')
    return name, int(source) if source.isdecimal() else source


class Camera:
    """Capture and encode away from Tk so a slow camera cannot freeze the UI."""

    def __init__(self, name, source, args, profile=None):
        self.name = name
        self.source = source
        self.args = args
        self.profile = profile
        self.settings = None
        self.lock = threading.Lock()
        self.frame = None
        self.status = 'Opening camera…'
        self.record_directory = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def snapshot(self):
        with self.lock:
            return self.frame, self.status

    def record_to(self, directory):
        with self.lock:
            self.record_directory = directory

    def read_settings(self, capture):
        controls = inspect_controls(self.source) if self.profile is not None else None
        requested = self.profile['controls'] if self.profile is not None else {}
        if controls is not None:
            for name, value in requested.items():
                if controls[name]['value'] != value:
                    raise RuntimeError(f'{name} changed since startup; restart to reapply configuration')
        fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
        return {
            'camera': self.name, 'source': self.source,
            'resolved_source': str(Path(self.source).resolve()) if isinstance(self.source, str) else None,
            'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
            'config_path': str(self.args.config) if self.profile is not None else None,
            'profile': self.profile,
            'setup_notes': getattr(self.args, 'setup_notes', None),
            'requested_capture': {'width': self.args.width, 'height': self.args.height, 'fps': self.args.fps},
            'actual_capture': {'width': capture.get(cv2.CAP_PROP_FRAME_WIDTH),
                               'height': capture.get(cv2.CAP_PROP_FRAME_HEIGHT),
                               'fps': capture.get(cv2.CAP_PROP_FPS),
                               'buffer_count': capture.get(cv2.CAP_PROP_BUFFERSIZE),
                               'pixel_format': ''.join(chr((fourcc >> (8 * i)) & 255) for i in range(4))},
            'actual_controls': controls,
            'opencv_version': cv2.__version__,
        }

    def run(self):
        capture = cv2.VideoCapture(self.source)
        writer = None
        recording_metadata = None
        metadata_path = None
        recording_frames = 0
        recording_started = None

        def finish_recording():
            nonlocal writer
            if writer is not None:
                writer.release()
                writer = None
                elapsed = time.monotonic() - recording_started
                recording_metadata['recording_summary'] = {
                    'frames': recording_frames, 'elapsed_seconds': elapsed,
                    'observed_fps': recording_frames / elapsed if elapsed else 0,
                    'finished_at_utc': datetime.now(timezone.utc).isoformat(),
                }
                metadata_path.write_text(json.dumps(recording_metadata, indent=2) + '\n')

        active_directory = None
        frame_times = deque()
        file_source = isinstance(self.source, str) and Path(self.source).is_file()
        try:
            if not capture.isOpened():
                raise RuntimeError(f'Cannot open {self.source}; check connection/access')
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
            capture.set(cv2.CAP_PROP_FPS, self.args.fps)
            # A single queued UVC buffer dropped alternate frames on this camera.
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 4)
            if self.profile is not None:
                apply_controls(self.source, self.profile['controls'])
            self.settings = self.read_settings(capture)
            actual_capture = self.settings['actual_capture']
            for name in ('width', 'height', 'fps'):
                if abs(actual_capture[name] - getattr(self.args, name)) > 0.1:
                    raise RuntimeError(f'{name}: requested {getattr(self.args, name)}, '
                                       f'camera negotiated {actual_capture[name]}')
            print(f'{self.name}: verified settings; '
                  f'{self.profile["controls"] if self.profile is not None else "no saved controls applied"}',
                  flush=True)
            fps = capture.get(cv2.CAP_PROP_FPS)
            if not 0 < fps <= 240:
                fps = self.args.fps
            next_frame = time.monotonic()
            while not self.stopping.is_set():
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError('No frames received; check connection or device selection')
                with self.lock:
                    directory = self.record_directory
                if directory != active_directory:
                    finish_recording()
                    active_directory = directory
                    if directory is not None:
                        filename = re.sub(r'[^A-Za-z0-9_.-]+', '_', self.name)
                        path = directory / f'{filename}.avi'
                        self.settings = self.read_settings(capture)
                        recording_metadata = dict(self.settings)
                        metadata_path = directory / f'{filename}.settings.json'
                        metadata_path.write_text(json.dumps(recording_metadata, indent=2) + '\n')
                        height, width = frame.shape[:2]
                        writer = cv2.VideoWriter(
                            str(path), cv2.VideoWriter_fourcc(*'MJPG'), fps, (width, height)
                        )
                        if not writer.isOpened():
                            writer.release()
                            writer = None
                            raise RuntimeError(f'Cannot create recording: {path}')
                        recording_frames = 0
                        recording_started = time.monotonic()
                        print(f'Recording {self.name}: {path}', flush=True)
                if writer is not None:
                    writer.write(frame)
                    recording_frames += 1
                now = time.monotonic()
                frame_times.append(now)
                while len(frame_times) > 2 and frame_times[0] < now - 2:
                    frame_times.popleft()
                measured_fps = ((len(frame_times) - 1) / (frame_times[-1] - frame_times[0])
                                if len(frame_times) > 1 else 0.0)
                state = 'REC' if writer is not None else 'Live'
                if self.profile is not None:
                    exposure = self.profile['controls'].get('exposure_absolute')
                    state += f' | {exposure / 10:g} ms manual' if exposure is not None else ' | configured'
                with self.lock:
                    self.frame = frame
                    self.status = f'{state} | {frame.shape[1]}×{frame.shape[0]} | {measured_fps:.1f} fps'
                # Live reads are paced by the device. Only files need playback pacing.
                if file_source:
                    next_frame += 1 / fps
                    now = time.monotonic()
                    if next_frame < now:
                        next_frame = now
                    self.stopping.wait(max(0, next_frame - now))
        except Exception as error:
            with self.lock:
                self.status = f'Error: {error}'
                self.frame = None
            print(f'{self.name}: {error}', flush=True)
        finally:
            try:
                finish_recording()
            finally:
                capture.release()


class Viewer:
    def __init__(self, root, cameras, args):
        # A large OpenCV thread pool adds overhead for small webcam images.
        # Configure before capture workers start; this affects only this process.
        cv2.setNumThreads(1)
        self.root = root
        self.cameras = cameras
        self.args = args
        self.recording = False
        self.closing = False
        self.images = {}
        self.rendered_frames = {}
        self.messages = {}
        self.startup_index = 0
        self.starting_camera = None
        self.startup_deadline = 0
        root.title('SO-101 cameras')
        root.geometry('1200x700')
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.bind('<Key-r>', lambda event: self.toggle_recording())
        root.bind('<Escape>', lambda event: self.close())
        toolbar = ttk.Frame(root, padding=8)
        toolbar.pack(fill='x')
        self.button = ttk.Button(toolbar, text='Record all (R)', command=self.toggle_recording)
        self.button.pack(side='left')
        self.message = ttk.Label(toolbar, text='Preview only | R: record/stop | Esc: quit')
        self.message.pack(side='left', padx=12)
        grid = ttk.Frame(root, padding=8)
        grid.pack(fill='both', expand=True)
        self.panels = []
        columns = min(2, len(cameras))
        for index, camera in enumerate(cameras):
            row, column = divmod(index, columns)
            grid.columnconfigure(column, weight=1, uniform='camera')
            grid.rowconfigure(row, weight=1, uniform='camera')
            panel = ttk.LabelFrame(grid, text=f'{camera.name} ({camera.source})', padding=4)
            panel.grid(row=row, column=column, sticky='nsew', padx=4, pady=4)
            status = ttk.Label(panel, text='Opening camera…')
            status.pack(side='bottom', fill='x')
            preview = tk.Label(panel, bg='#171717')
            preview.pack(fill='both', expand=True)
            self.panels.append((preview, status))
        self.start_next_camera()
        self.refresh()

    def start_next_camera(self):
        """Let each stream start before opening the next device on a shared hub."""
        if self.closing:
            return
        camera = self.starting_camera
        if (camera is not None and camera.thread.is_alive() and camera.snapshot()[0] is None
                and time.monotonic() < self.startup_deadline):
            self.root.after(20, self.start_next_camera)
            return
        if self.startup_index < len(self.cameras):
            camera = self.cameras[self.startup_index]
            self.startup_index += 1
            self.starting_camera = camera
            self.startup_deadline = time.monotonic() + 5
            camera.thread.start()
            self.root.after(20, self.start_next_camera)

    def toggle_recording(self):
        if self.closing:
            return
        directory = None
        if not self.recording:
            unavailable = [camera.name for camera in self.cameras
                           if camera.snapshot()[0] is None or not camera.thread.is_alive()]
            if unavailable:
                self.message.configure(text=f'Cannot record; cameras not ready: {", ".join(unavailable)}')
                return
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            directory = self.args.output_dir.expanduser().resolve() / stamp
            try:
                directory.mkdir(parents=True, exist_ok=False)
            except OSError as error:
                self.message.configure(text=f'Cannot start recording: {error}')
                return
        for camera in self.cameras:
            camera.record_to(directory)
        self.recording = not self.recording
        self.button.configure(text='Stop recording (R)' if self.recording else 'Record all (R)')
        self.message.configure(text=f'Recording requested: {directory}' if self.recording
                               else 'Recording stopped | Preview only')

    def refresh(self):
        if self.closing:
            return
        started = time.monotonic()
        for index, (camera, (preview, status)) in enumerate(zip(self.cameras, self.panels)):
            frame, message = camera.snapshot()
            if message != self.messages.get(index):
                status.configure(text=message)
                self.messages[index] = message
            if frame is None:
                if index in self.images:
                    preview.configure(image='')
                    self.images.pop(index, None)
                    self.rendered_frames.pop(index, None)
                continue
            bounds = (max(1, preview.winfo_width()), max(1, preview.winfo_height()))
            previous = self.rendered_frames.get(index)
            if previous is not None and frame is previous[0] and bounds == previous[1]:
                continue
            image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            image.thumbnail(bounds, Image.Resampling.BILINEAR)
            self.images[index] = ImageTk.PhotoImage(image)
            preview.configure(image=self.images[index])
            self.rendered_frames[index] = (frame, bounds)
        # Include rendering time in the polling interval, rather than adding it.
        delay_ms = max(1, 16 - int((time.monotonic() - started) * 1000))
        self.root.after(delay_ms, self.refresh)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.button.configure(state='disabled')
        self.message.configure(text='Closing cameras and saving recordings…')
        for camera in self.cameras:
            camera.stopping.set()
        self.finish_close()

    def finish_close(self):
        if any(camera.thread.is_alive() for camera in self.cameras):
            self.root.after(50, self.finish_close)
        else:
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', action='append', type=camera_argument,
                        help='Repeat NAME=SOURCE (device index, /dev/video path, or video file)')
    parser.add_argument('--list', action='store_true', help='List detected device paths and exit')
    config_group = parser.add_mutually_exclusive_group()
    config_group.add_argument('--config', type=Path, default=DEFAULT_CONFIG,
                              help='Saved per-camera settings (default: experiments/camera_settings.json)')
    config_group.add_argument('--no-config', action='store_true',
                              help='Discover cameras without applying saved image controls')
    parser.add_argument('--width', type=int, help='Override capture width (config default: 640)')
    parser.add_argument('--height', type=int, help='Override capture height (config default: 480)')
    parser.add_argument('--fps', type=float, help='Override capture fps (config default: 30)')
    parser.add_argument('--output-dir', type=Path,
                        default=Path(__file__).resolve().parent / 'camera_recordings')
    args = parser.parse_args()
    if args.list:
        detected = discover_cameras()
        for name, source in detected:
            print(f'{name}={source}')
        if not detected:
            print('No camera devices detected.')
        return
    config = {}
    if not args.no_config:
        try:
            args.config = args.config.expanduser().resolve()
            config = load_config(args.config)
        except (OSError, ValueError, TypeError, KeyError) as error:
            parser.error(f'Cannot load camera configuration: {error}')
    for name, default in [('width', 640), ('height', 480), ('fps', 30)]:
        if getattr(args, name) is None:
            setattr(args, name, config.get('capture', {}).get(name, default))
    if args.width <= 0 or args.height <= 0 or not 0 < args.fps <= 240:
        parser.error('width/height must be positive; fps must be between 0 and 240')
    profiles = config.get('cameras', {})
    args.setup_notes = config.get('setup_notes')
    sources = args.camera or ([(role, profile['device']) for role, profile in profiles.items()]
                              if profiles else discover_cameras())
    if profiles:
        for name, source in sources:
            if name not in profiles:
                parser.error(f'No profile for {name}; use a configured role or --no-config')
            if not isinstance(source, str) or not Path(source).exists():
                parser.error(f'{name}: device {source} missing; check --list and update {args.config}')
    if not sources:
        parser.error('No cameras detected. Connect cameras or pass --camera NAME=/dev/video0')
    filenames = [re.sub(r'[^A-Za-z0-9_.-]+', '_', name) for name, _ in sources]
    if len(set(filenames)) != len(filenames):
        parser.error('Camera names must be unique after replacing punctuation with underscores')
    root = tk.Tk()
    viewer = Viewer(root, [Camera(name, source, args, profiles.get(name)) for name, source in sources], args)
    signal.signal(signal.SIGTERM, lambda *_: root.after(0, viewer.close))
    signal.signal(signal.SIGINT, lambda *_: root.after(0, viewer.close))
    root.mainloop()


if __name__ == '__main__':
    main()
