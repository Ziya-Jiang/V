"""Resident local SigLIP: masked RGB crops, image identity and text scores.

All model/tokenizer loads are local_files_only. No generative model or API.
One image encoding per candidate per frame; cached text embeddings across runs.
"""
from pathlib import Path
import time
import cv2
import numpy as np

BOOK_TEXTS = ['a photo of a book.', 'a photo of a book spine.', 'a photo of a book cover.']
OTHER_TEXTS = ['a photo of a cart handle.', 'a photo of a metal cart frame.',
              'a photo of a chair.', 'a photo of a robot gripper.', 'a photo of an empty shelf.']
_MODELS = {}


def model_path(config):
    path = Path(config['semantic_model_path']).expanduser()
    if not (path/'config.json').is_file() and (path/'snapshots').is_dir():
        snapshots = sorted(p for p in (path/'snapshots').iterdir() if (p/'model.safetensors').is_file())
        if not snapshots: raise ValueError('Local SigLIP checkpoint is incomplete')
        path = snapshots[-1]
    for name in ('model.safetensors', 'config.json', 'tokenizer.json', 'preprocessor_config.json'):
        if not (path/name).is_file(): raise ValueError(f'Local SigLIP file missing: {path/name}')
    return path


class LocalSemantics:
    def __init__(self, config):
        import torch
        from transformers import SiglipModel, SiglipImageProcessor, PreTrainedTokenizerFast
        self.torch = torch
        requested = config['semantic_device']
        self.device = ('cuda' if torch.cuda.is_available() else 'cpu') if requested == 'auto' else requested
        if self.device == 'cuda' and not torch.cuda.is_available():
            raise ValueError('CUDA is unavailable for the configured local semantic model')
        path = model_path(config)
        self.model = SiglipModel.from_pretrained(str(path), local_files_only=True).eval().to(self.device)
        self.processor = SiglipImageProcessor.from_pretrained(str(path), local_files_only=True)
        # The installed fast tokenizer JSON includes SigLIP's normalization/EOS
        # processing. Loading it directly avoids a SentencePiece dependency.
        self.tokenizer = PreTrainedTokenizerFast(tokenizer_file=str(path/'tokenizer.json'),
            eos_token='</s>', unk_token='<unk>', pad_token='</s>', model_max_length=64)
        self.text_cache = {}
        self.batch_size = config['semantic_batch_size']

    def text_features(self, texts):
        missing = list(dict.fromkeys(text for text in texts if text not in self.text_cache))
        if missing:
            ids = self.tokenizer(missing, padding='max_length', truncation=True, max_length=64,
                                 return_tensors='pt')['input_ids'].to(self.device)
            with self.torch.inference_mode():
                features = self.model.get_text_features(input_ids=ids)
                features = self.torch.nn.functional.normalize(features, dim=-1).cpu().numpy()
            self.text_cache.update(zip(missing, features))
        return np.stack([self.text_cache[text] for text in texts])

    def encode(self, crops, texts, checkpoint=lambda: None):
        if not crops: return np.empty((0, self.model.config.text_config.hidden_size)), np.empty((0, len(texts)))
        embeddings = []
        for start in range(0, len(crops), self.batch_size):
            checkpoint()
            pixels = self.processor(images=crops[start:start+self.batch_size], return_tensors='pt')['pixel_values'].to(self.device)
            with self.torch.inference_mode():
                features = self.model.get_image_features(pixel_values=pixels)
                embeddings.append(self.torch.nn.functional.normalize(features, dim=-1).cpu().numpy())
            checkpoint()
        images = np.concatenate(embeddings)
        text = self.text_features(texts)
        scale = float(self.model.logit_scale.detach().exp().cpu())
        bias = float(self.model.logit_bias.detach().cpu())
        logits = images @ text.T * scale + bias
        checkpoint()
        return images, logits


def get_model(config):
    key = (str(model_path(config)), config['semantic_device'], config['semantic_batch_size'])
    if key not in _MODELS: _MODELS[key] = LocalSemantics(config)
    return _MODELS[key]


def crops_for(rgb, items):
    result = []
    for item in items:
        mask = item['book_mask']
        ys, xs = np.nonzero(mask)
        x0, x1, y0, y1 = xs.min(), xs.max()+1, ys.min(), ys.max()+1
        crop = rgb[y0:y1, x0:x1].copy()
        crop[~mask[y0:y1, x0:x1]] = 127
        result.append(crop)
    return result


def annotate_views(head_rgb, head, wrist_rgb, wrist, rule, config, checkpoint=lambda: None, model=None):
    """Mutate per-view observations with scores only; never add wrist geometry."""
    started = time.monotonic()
    model = get_model(config) if model is None else model
    query = rule['query']
    prompts = BOOK_TEXTS + OTHER_TEXTS + [query]
    crops = crops_for(head_rgb, head)+crops_for(wrist_rgb, wrist)
    embeddings, logits = model.encode(crops, prompts, checkpoint)
    items = head+wrist
    if (logits.shape != (len(items), len(prompts)) or embeddings.ndim!=2 or embeddings.shape[0] != len(items)
            or not np.isfinite(logits).all() or not np.isfinite(embeddings).all()):
        raise ValueError('Invalid local semantic outputs')
    for item, embedding, row in zip(items, embeddings, logits):
        book = float(np.max(row[:len(BOOK_TEXTS)]))
        other = float(np.max(row[len(BOOK_TEXTS):-1]))
        margin = book-other
        item.update(semantic_embedding=embedding, semantic_book_margin=margin,
                    semantic_query_logit=float(row[-1]), verified_book=margin >= config['semantic_book_margin'],
                    semantic_other_label=OTHER_TEXTS[int(np.argmax(row[len(BOOK_TEXTS):-1]))])
    # Category transfer is weaker than identity: a partly visible head book may
    # resemble a confirmed wrist book without proving which wrist ID it is.
    # Only originally positive opposite-view evidence can support this step.
    positive_head=[v for v in head if v['verified_book']]
    positive_wrist=[v for v in wrist if v['verified_book']]
    for view, opposite in ((head,positive_wrist),(wrist,positive_head)):
        for item in view:
            if item['verified_book'] or not opposite: continue
            similarity=max(float(item['semantic_embedding']@v['semantic_embedding']) for v in opposite)
            if (item['semantic_book_margin']>=config['semantic_category_floor'] and
                    similarity>=config['semantic_category_transfer_min']):
                item.update(verified_book=True,category_evidence='opposite_view_book_category',
                            category_support_similarity=similarity)
    return dict(local_semantics_s=time.monotonic()-started, model=config['semantic_model_path'],
                candidates=[dict(id=c['id'], is_book=bool(c['verified_book']),
                    book_margin=c['semantic_book_margin'], query_logit=c['semantic_query_logit'],
                    category_evidence=c.get('category_evidence','own_view'),
                    competing_label=c['semantic_other_label']) for c in items])
