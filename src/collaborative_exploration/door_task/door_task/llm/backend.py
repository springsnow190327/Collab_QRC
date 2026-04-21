"""LLM backend wrapper + JSON parser.

Thin shim over ``vlm_explorer.vlm_backends.query_vlm`` so the rest of
the package never imports it directly. Keeps a single seam to swap in a
mock backend or a different provider later.
"""

from __future__ import annotations

import json
import re

_JSON_RE = re.compile(r"\{.*\}", re.DOTALL)


def call_vlm(
    provider: str,
    model: str,
    system_prompt: str,
    user_prompt: str,
    image_b64: str,
    *,
    temperature: float,
    max_tokens: int,
    timeout: float,
) -> str:
    from vlm_explorer.vlm_backends import query_vlm  # noqa: PLC0415
    return query_vlm(
        provider=provider,
        model=model,
        system_prompt=system_prompt,
        user_prompt=user_prompt,
        image_b64=image_b64,
        temperature=temperature,
        max_tokens=max_tokens,
        timeout_sec=timeout,
    )


def parse_action_json(raw: str) -> dict:
    """Extract the first {...} JSON object from a possibly-noisy LLM reply."""
    m = _JSON_RE.search(raw)
    if not m:
        raise ValueError(f"no JSON object found in: {raw[:200]}")
    return json.loads(m.group(0))
