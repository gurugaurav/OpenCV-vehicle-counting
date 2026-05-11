"""Stream source resolution — RTSP passthrough, YouTube → direct URL via yt-dlp."""

import re
import subprocess
import sys


def is_youtube(source: str) -> bool:
    return bool(re.search(r'(youtube\.com|youtu\.be)/', str(source)))


def is_live(source: str) -> bool:
    """True for RTSP/RTMP streams and YouTube URLs (as opposed to local files)."""
    s = str(source).lower()
    return s.startswith(('rtsp://', 'rtsps://', 'rtmp://')) or is_youtube(source)


def resolve_source(source: str) -> str:
    """Return a URL that OpenCV/YOLO can open directly.

    RTSP/RTMP and file paths are returned unchanged.
    YouTube URLs are resolved to a direct CDN stream URL via yt-dlp.
    """
    if is_youtube(source):
        return _yt_stream_url(source)
    return source


def _yt_stream_url(url: str) -> str:
    try:
        r = subprocess.run(
            [
                'yt-dlp',
                '-f', 'best[height<=720][ext=mp4]/best[height<=720]/best',
                '-g', '--no-playlist',
                url,
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )
    except FileNotFoundError:
        sys.exit('[error] yt-dlp not found — install with: pip install yt-dlp')
    except subprocess.TimeoutExpired:
        sys.exit('[error] yt-dlp timed out resolving YouTube URL')

    if r.returncode != 0:
        sys.exit(f'[error] yt-dlp: {r.stderr.strip()}')

    stream_url = r.stdout.strip().split('\n')[0]
    if not stream_url:
        sys.exit('[error] yt-dlp returned an empty URL')

    print('[info] YouTube stream resolved.')
    return stream_url
