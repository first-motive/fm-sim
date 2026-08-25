#!/usr/bin/env python3
# fm-render: render-check sha256:405935ed00b9cfe5a28a712042b4c00c4614d71aad44333bbea056d4ca1923d0 — rendered by the First Motive render plane — edit the upstream source, not this file
"""Verify that this repo's rendered artifacts still match what the plane rendered.

Some files here are not authored in this repo. They are rendered from a single
upstream source shared across First Motive repos, and each one carries a stamp:
a hash of its own body. This file is rendered too, and stamped like the rest.

It is stdlib-only and takes no arguments, so a CI job can run it with nothing
installed and no access to the upstream source:

    python3 .fm/render-check.py

It reads `.fm/render.lock.json` — the per-repo record of which artifacts were
rendered here and what each one hashed to — and fails when a rendered file was
hand-edited. Staleness (the upstream source moved on) is not detectable from
inside a consumer and is not this script's job: the plane opens a sync PR when a
source changes.

The plane imports the functions below directly from its own copy of this file,
so the stamp format has exactly one implementation on both sides.
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

# Marker vocabulary. `hash` stamps a single line into the file itself; `html` and
# `sh` fence a block inside a host file the repo also authors; `json` stamps a
# reserved top-level key, since JSON carries no comments.
STAMP_PREFIX = "fm-render:"
BLOCK_BEGIN = "fm-render:begin"
BLOCK_END = "fm-render:end"
JSON_KEY = "_fm_render"
# How each block style writes a comment. A block artifact lands inside a file the
# consumer owns, so the markers have to be comments in that file's own language.
BLOCK_COMMENT = {"html": ("<!-- ", " -->"), "sh": ("# ", "")}
# How each whole-file style writes its one stamp line. A rendered file carries the
# stamp in its own comment syntax, which is why the style is named per artifact
# rather than assumed: `#` is a comment in a shell script and a syntax error in a
# stylesheet.
FILE_COMMENT = {"hash": ("#", ""), "slash": ("//", ""), "css": ("/*", " */")}
ORIGIN = "rendered by the First Motive render plane — edit the upstream source, not this file"
LOCK_PATH = ".fm/render.lock.json"
LOCK_ID = "render-lock"


class DriftError(Exception):
    """A rendered artifact does not match its stamp."""


def digest(body: str) -> str:
    """Hash a rendered body. The stamp itself is never part of the hash."""
    return hashlib.sha256(body.encode("utf-8")).hexdigest()


def canonical_json(doc: dict) -> str:
    """Serialize a JSON artifact the one way the plane serializes it."""
    return json.dumps(doc, indent=2, sort_keys=True) + "\n"


def normalize(body: str) -> str:
    """One trailing newline, so a stray blank line is not read as an edit."""
    return body.rstrip("\n") + "\n"


# --- stamping -------------------------------------------------------------


def stamp_file(artifact_id: str, body: str, style: str = "hash") -> str:
    """Render a whole-file artifact: body plus one stamp line.

    A shebang keeps line 1 — the stamp goes under it — so a rendered script
    stays executable. ``style`` picks the comment syntax (see FILE_COMMENT); an
    unknown one falls back to ``#`` rather than raising, because a consumer
    running an older check script must still be able to read a newer stamp.
    """
    body = normalize(body)
    open_mark, close_mark = FILE_COMMENT.get(style, FILE_COMMENT["hash"])
    line = f"{open_mark} {STAMP_PREFIX} {artifact_id} sha256:{digest(body)} — {ORIGIN}{close_mark}\n"
    lines = body.splitlines(keepends=True)
    if lines and lines[0].startswith("#!"):
        return lines[0] + line + "".join(lines[1:])
    return line + body


def unstamp_file(text: str) -> tuple[str, str]:
    """Recover (artifact_id, body) from a stamped file. Inverse of stamp_file."""
    lines = text.splitlines(keepends=True)
    for index, line in enumerate(lines[:2]):
        if STAMP_PREFIX in line and BLOCK_BEGIN not in line:
            artifact_id = line.split(STAMP_PREFIX, 1)[1].split()[0]
            return artifact_id, "".join(lines[:index] + lines[index + 1 :])
    raise DriftError("no fm-render stamp in the first two lines")


def block_indent(body: str) -> str:
    """The indentation a block's markers must share with its body.

    A `sh` block often lands inside indented YAML — a workflow step under
    `steps:`. A marker at column zero there is a syntax error, so the markers
    take the indentation of the body's own first line.
    """
    for line in body.splitlines():
        if line.strip():
            return line[: len(line) - len(line.lstrip())]
    return ""


def stamp_block(artifact_id: str, body: str, style: str = "html") -> str:
    """Render a block artifact: the body fenced by begin/end markers."""
    body = normalize(body)
    open_mark, close_mark = BLOCK_COMMENT[style]
    pad = block_indent(body)
    return (
        f"{pad}{open_mark}{BLOCK_BEGIN} {artifact_id} sha256:{digest(body)} — {ORIGIN}{close_mark}\n"
        f"{body}"
        f"{pad}{open_mark}{BLOCK_END} {artifact_id}{close_mark}\n"
    )


def find_block(text: str, artifact_id: str) -> tuple[int, int] | None:
    """Locate a block artifact's marker lines as (begin index, end index)."""
    begin = end = None
    for index, line in enumerate(text.splitlines()):
        if f"{BLOCK_BEGIN} {artifact_id} " in line:
            begin = index
        elif f"{BLOCK_END} {artifact_id}" in line:
            end = index
    if begin is None or end is None or end < begin:
        return None
    return begin, end


def unstamp_block(text: str, artifact_id: str) -> str:
    """Recover a block artifact's body from its host file."""
    span = find_block(text, artifact_id)
    if span is None:
        raise DriftError(f"no fm-render block for {artifact_id}")
    begin, end = span
    lines = text.splitlines(keepends=True)
    return "".join(lines[begin + 1 : end])


def stamp_json(artifact_id: str, body: str) -> str:
    """Render a JSON artifact: the reserved key carries what a comment cannot."""
    doc = json.loads(body)
    doc.pop(JSON_KEY, None)
    stamped = {JSON_KEY: {"artifact": artifact_id, "sha256": digest(canonical_json(doc)), "origin": ORIGIN}}
    stamped.update(doc)
    return canonical_json(stamped)


def unstamp_json(text: str) -> tuple[str, str]:
    """Recover (artifact_id, canonical body) from a stamped JSON artifact."""
    doc = json.loads(text)
    stamp = doc.pop(JSON_KEY, None)
    if not isinstance(stamp, dict) or "artifact" not in stamp:
        raise DriftError(f"no {JSON_KEY} key")
    return stamp["artifact"], canonical_json(doc)


def stamped_digest(text: str, stamp_style: str, artifact_id: str) -> str:
    """Read back the digest a rendered artifact claims for itself."""
    if stamp_style == "json":
        return json.loads(text)[JSON_KEY]["sha256"]
    if stamp_style in BLOCK_COMMENT:
        span = find_block(text, artifact_id)
        if span is None:
            raise DriftError(f"no fm-render block for {artifact_id}")
        line = text.splitlines()[span[0]]
    else:
        line = next((one for one in text.splitlines()[:2] if STAMP_PREFIX in one), "")
    for token in line.split():
        if token.startswith("sha256:"):
            return token.split(":", 1)[1]
    raise DriftError("stamp carries no sha256")


def actual_digest(text: str, stamp_style: str, artifact_id: str) -> str:
    """Hash what the artifact actually contains right now."""
    if stamp_style == "json":
        return digest(unstamp_json(text)[1])
    if stamp_style in BLOCK_COMMENT:
        return digest(normalize(unstamp_block(text, artifact_id)))
    return digest(normalize(unstamp_file(text)[1]))


# --- consumer-side check --------------------------------------------------


def check_repo(root: Path) -> list[str]:
    """Return one message per rendered artifact that no longer matches its stamp."""
    lock_file = root / LOCK_PATH
    if not lock_file.exists():
        return [f"{LOCK_PATH}: missing — this repo has no rendered artifacts recorded"]

    lock_text = lock_file.read_text()
    problems: list[str] = []
    try:
        if stamped_digest(lock_text, "json", LOCK_ID) != actual_digest(lock_text, "json", LOCK_ID):
            problems.append(f"{LOCK_PATH}: hand-edited")
    except (DriftError, KeyError, ValueError) as error:
        return [f"{LOCK_PATH}: unreadable ({error})"]

    for artifact_id, entry in sorted(json.loads(lock_text)["artifacts"].items()):
        # One artifact can land in more than one host here: the bootstrap
        # preamble is the same fact in both install.sh and run.sh.
        for relative in entry["paths"]:
            path = root / relative
            if not path.exists():
                problems.append(f"{relative}: missing — rendered artifact was deleted")
                continue
            text = path.read_text()
            try:
                claimed = stamped_digest(text, entry["stamp"], artifact_id)
                actual = actual_digest(text, entry["stamp"], artifact_id)
            except (DriftError, KeyError, ValueError) as error:
                problems.append(f"{relative}: stamp unreadable ({error})")
                continue
            if claimed != actual:
                problems.append(f"{relative}: hand-edited — content does not match its stamp")
            elif claimed != entry["sha256"]:
                problems.append(f"{relative}: stamp rewritten — does not match {LOCK_PATH}")

    return problems


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    problems = check_repo(root)
    if problems:
        print("render drift — a rendered artifact was edited in place:", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        print(
            "\nFix it in the upstream source and let the sync PR bring it here.",
            file=sys.stderr,
        )
        return 1
    print("render check: every rendered artifact matches its stamp")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
