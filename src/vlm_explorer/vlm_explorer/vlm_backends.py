from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request


class VLMBackendError(RuntimeError):
    pass


def resolve_provider(provider: str) -> str:
    p = provider.strip().lower()
    if p and p != "auto":
        return p
    if os.environ.get("SILICONFLOW_API_KEY"):
        return "siliconflow"
    if os.environ.get("XAI_API_KEY"):
        return "xai"
    if os.environ.get("OPENAI_API_KEY"):
        return "openai"
    if os.environ.get("ANTHROPIC_API_KEY"):
        return "anthropic"
    return "none"


def _clean(s: str) -> str:
    """Strip any whitespace (including embedded newlines from line-wrapped
    pastes) from an API key. Nothing legal in a bearer token is whitespace,
    so it's safe to rip all of it out."""
    return "".join(s.split()) if s else ""


def resolve_api_key(provider: str) -> str:
    p = resolve_provider(provider)
    if p == "siliconflow":
        return _clean(os.environ.get("SILICONFLOW_API_KEY", ""))
    if p == "xai":
        return _clean(os.environ.get("XAI_API_KEY", ""))
    if p == "openai":
        return _clean(os.environ.get("OPENAI_API_KEY", ""))
    if p == "anthropic":
        return _clean(os.environ.get("ANTHROPIC_API_KEY", ""))
    return ""


def extract_json_object(raw: str) -> dict | None:
    text = raw.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if len(lines) >= 2:
            text = "\n".join(lines[1:])
        if text.endswith("```"):
            text = text[:-3]
    try:
        return json.loads(text.strip())
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", text, re.DOTALL)
        if not match:
            return None
        try:
            return json.loads(match.group(0))
        except json.JSONDecodeError:
            return None


def _post_json(url: str, payload: dict, headers: dict[str, str], timeout_sec: float) -> dict:
    request = urllib.request.Request(
        url=url,
        data=json.dumps(payload).encode("utf-8"),
        headers=headers,
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise VLMBackendError(f"HTTP {exc.code}: {detail[:600]}") from exc
    except urllib.error.URLError as exc:
        raise VLMBackendError(f"Request failed: {exc.reason}") from exc


def query_vlm(
    provider: str,
    model: str,
    system_prompt: str,
    user_prompt: str,
    image_b64: str,
    *,
    temperature: float,
    max_tokens: int,
    timeout_sec: float,
) -> str:
    p = resolve_provider(provider)
    api_key = resolve_api_key(p)
    if not api_key:
        raise VLMBackendError(f"No API key available for provider '{p}'")

    if p == "anthropic":
        payload = {
            "model": model,
            "max_tokens": max_tokens,
            "system": system_prompt,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image",
                            "source": {
                                "type": "base64",
                                "media_type": "image/jpeg",
                                "data": image_b64,
                            },
                        },
                        {"type": "text", "text": user_prompt},
                    ],
                }
            ],
            "temperature": temperature,
        }
        body = _post_json(
            "https://api.anthropic.com/v1/messages",
            payload,
            {
                "Content-Type": "application/json",
                "x-api-key": api_key,
                "anthropic-version": "2023-06-01",
            },
            timeout_sec,
        )
        content = body.get("content", [])
        if not content:
            raise VLMBackendError("Anthropic response contained no content")
        return str(content[0].get("text", ""))

    if p in {"openai", "xai", "siliconflow"}:
        url = "https://api.openai.com/v1/chat/completions"
        auth_key = api_key
        if p == "xai":
            url = os.environ.get("XAI_API_BASE_URL", "https://api.x.ai/v1").rstrip("/") + "/chat/completions"
        elif p == "siliconflow":
            url = os.environ.get(
                "SILICONFLOW_API_BASE_URL", "https://api.siliconflow.com/v1"
            ).rstrip("/") + "/chat/completions"
        payload = {
            "model": model,
            "temperature": temperature,
            "max_tokens": max_tokens,
            "messages": [
                {"role": "system", "content": system_prompt},
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_b64}",
                                "detail": "low",
                            },
                        },
                        {"type": "text", "text": user_prompt},
                    ],
                },
            ],
        }
        body = _post_json(
            url,
            payload,
            {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {auth_key}",
            },
            timeout_sec,
        )
        choices = body.get("choices", [])
        if not choices:
            raise VLMBackendError(f"{p} response contained no choices")
        return str(choices[0].get("message", {}).get("content", ""))

    raise VLMBackendError(f"Unsupported provider '{provider}'")
