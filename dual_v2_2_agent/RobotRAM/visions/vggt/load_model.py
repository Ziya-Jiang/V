import os
import torch

from visions.vggt.vggt.models.vggt import VGGT


def _clear_broken_proxy_env():
    """Broken proxy settings cause HuggingFace hub requests to fail."""
    for key in (
        "HTTP_PROXY", "HTTPS_PROXY", "http_proxy", "https_proxy",
        "ALL_PROXY", "all_proxy", "SOCKS_PROXY", "SOCKS5_PROXY",
        "socks_proxy", "socks5_proxy",
    ):
        os.environ.pop(key, None)


def load_vggt_model(model_name_or_path="facebook/VGGT-1B", local_files_only=True):
    """Load VGGT from HuggingFace cache or a local path/state-dict file."""
    _clear_broken_proxy_env()
    model_name_or_path = os.environ.get("VGGT_MODEL_PATH", model_name_or_path)

    if os.path.isfile(model_name_or_path):
        model = VGGT()
        state_dict = torch.load(model_name_or_path, map_location="cpu")
        if isinstance(state_dict, dict):
            for key in ("state_dict", "model", "module"):
                if key in state_dict and isinstance(state_dict[key], dict):
                    state_dict = state_dict[key]
                    break
        model.load_state_dict(state_dict)
        return model

    if local_files_only:
        os.environ.setdefault("HF_HUB_OFFLINE", "1")

    try:
        return VGGT.from_pretrained(model_name_or_path, local_files_only=local_files_only)
    except TypeError:
        if local_files_only:
            os.environ.setdefault("HF_HUB_OFFLINE", "1")
        return VGGT.from_pretrained(model_name_or_path)
    except Exception as exc:
        raise RuntimeError(
            "Failed to load VGGT. If Hugging Face is unreachable, ensure "
            "facebook/VGGT-1B is cached locally or set VGGT_MODEL_PATH to a "
            "local model directory/state-dict file."
        ) from exc
