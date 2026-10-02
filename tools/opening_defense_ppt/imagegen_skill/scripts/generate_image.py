#!/usr/bin/env python3
"""Generate one raster image through an OpenAI-compatible image endpoint."""

from __future__ import annotations

import argparse
import base64
import json
import os
import ssl
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--prompt", help="Prompt text.")
    source.add_argument("--prompt-file", type=Path, help="UTF-8 prompt file.")
    parser.add_argument("--out", type=Path, required=True, help="Output image path.")
    parser.add_argument("--model", default="gpt-image-2")
    parser.add_argument("--size", default="1536x1024")
    parser.add_argument("--quality", default="high")
    parser.add_argument("--n", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=180.0)
    return parser.parse_args()


def read_prompt(args: argparse.Namespace) -> str:
    if args.prompt is not None:
        return args.prompt
    return args.prompt_file.read_text(encoding="utf-8")


def endpoint_from_gateway(gateway: str) -> str:
    base = gateway.rstrip("/")
    if base.endswith("/images/generations"):
        return base
    return f"{base}/images/generations"


def request_image(prompt: str, args: argparse.Namespace) -> dict:
    api_key = os.environ.get("IMAGE_GEN_API_KEY")
    if not api_key:
        raise RuntimeError("IMAGE_GEN_API_KEY is not set in the local environment")
    gateway = os.environ.get(
        "GATEWAY",
        "https://api.xcdw.us/v1/images/generations",
    )
    payload = {
        "model": args.model,
        "prompt": prompt,
        "size": args.size,
        "quality": args.quality,
        "n": args.n,
    }
    request = urllib.request.Request(
        endpoint_from_gateway(gateway),
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )
    context = ssl.create_default_context()
    try:
        with urllib.request.urlopen(request, timeout=args.timeout, context=context) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"image API HTTP {exc.code}: {detail}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"image API connection failed: {exc}") from exc


def save_first_image(response: dict, output: Path, timeout: float) -> None:
    items = response.get("data")
    if not isinstance(items, list) or not items:
        raise RuntimeError(f"image API returned no image data: {json.dumps(response)[:1000]}")
    first = items[0]
    if not isinstance(first, dict):
        raise RuntimeError("image API returned malformed image data")

    output.parent.mkdir(parents=True, exist_ok=True)
    if first.get("b64_json"):
        output.write_bytes(base64.b64decode(first["b64_json"]))
        return
    if first.get("url"):
        request = urllib.request.Request(first["url"])
        with urllib.request.urlopen(request, timeout=timeout) as response_stream:
            output.write_bytes(response_stream.read())
        return
    raise RuntimeError("image API response contains neither b64_json nor url")


def main() -> int:
    args = parse_args()
    try:
        prompt = read_prompt(args)
        response = request_image(prompt, args)
        save_first_image(response, args.out, args.timeout)
    except Exception as exc:  # noqa: BLE001 - CLI should report a concise failure.
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
