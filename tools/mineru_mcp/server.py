#!/usr/bin/env python3
"""Small stdio MCP wrapper around the local MinerU CLI."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

from mcp.server.fastmcp import FastMCP


DEFAULT_MINERU = "/home/dream/Applications/MinerU/.venv/bin/mineru"

mcp = FastMCP(
    "fusionlynx-mineru",
    instructions=(
        "Parse local research PDFs with MinerU and return the generated "
        "Markdown artifact locations."
    ),
)


@mcp.tool()
def parse_pdf(
    input_path: str,
    output_path: str,
    backend: str = "pipeline",
    start_page: int | None = None,
    end_page: int | None = None,
) -> str:
    """Parse a local PDF with MinerU and return a concise run report."""
    source = Path(input_path).expanduser().resolve()
    output = Path(output_path).expanduser().resolve()
    mineru = Path(os.environ.get("MINERU_BIN", DEFAULT_MINERU)).expanduser()

    if not source.is_file():
        raise ValueError(f"input_path is not a file: {source}")
    if source.suffix.lower() != ".pdf":
        raise ValueError(f"input_path must be a PDF: {source}")
    if not mineru.is_file():
        raise RuntimeError(f"MinerU executable not found: {mineru}")
    if backend not in {"pipeline", "hybrid-engine", "vlm-engine"}:
        raise ValueError(
            "backend must be one of pipeline, hybrid-engine, vlm-engine"
        )

    output.mkdir(parents=True, exist_ok=True)
    command = [str(mineru), "-p", str(source), "-o", str(output), "-b", backend]
    if start_page is not None:
        command.extend(["-s", str(start_page)])
    if end_page is not None:
        command.extend(["-e", str(end_page)])

    completed = subprocess.run(
        command,
        check=False,
        text=True,
        capture_output=True,
        timeout=1800,
    )
    if completed.returncode != 0:
        tail = (completed.stderr or completed.stdout)[-4000:]
        raise RuntimeError(
            f"MinerU failed with exit code {completed.returncode}:\n{tail}"
        )

    markdown = sorted(output.rglob("*.md"))
    files = "\n".join(str(path) for path in markdown)
    return (
        f"MinerU completed successfully.\n"
        f"input: {source}\n"
        f"output: {output}\n"
        f"markdown_files:\n{files or '(none found)'}"
    )


if __name__ == "__main__":
    mcp.run("stdio")
