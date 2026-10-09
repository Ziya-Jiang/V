"""Optional LLM backend for the Agent's open-ended planning.

Kept separate so the rule-based path (and the whole framework) imports with zero
LLM dependencies. Provide ``OPENAI_BASE_URL`` / ``OPENAI_API_KEY`` / ``LLM_MODEL``
(or a custom ``llm.chat(prompt) -> str`` object) to enable open-ended requests.
"""

from __future__ import annotations

import os
from typing import Optional


class OpenAICompatLLM:
    """Minimal OpenAI-compatible chat client exposing ``chat(prompt) -> str``."""

    def __init__(self, base_url: Optional[str] = None, api_key: Optional[str] = None, model: Optional[str] = None) -> None:
        self.base_url = base_url or os.environ.get("OPENAI_BASE_URL")
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        self.model = model or os.environ.get("LLM_MODEL", "gpt-4o")
        if not self.base_url or not self.api_key:
            raise ValueError("OPENAI_BASE_URL and OPENAI_API_KEY are required")

        from openai import OpenAI  # imported lazily; only needed for this backend

        self.client = OpenAI(base_url=self.base_url, api_key=self.api_key)

    def chat(self, prompt: str) -> str:
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.0,
        )
        return response.choices[0].message.content


def make_llm_from_env() -> Optional[OpenAICompatLLM]:
    """Return an LLM backend when credentials are present, else ``None``."""
    if os.environ.get("OPENAI_BASE_URL") and os.environ.get("OPENAI_API_KEY"):
        return OpenAICompatLLM()
    return None
