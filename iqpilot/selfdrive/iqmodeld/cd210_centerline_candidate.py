"""Select an installed, verified split-model candidate through existing Params."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

CANDIDATE_REF = "cd210-centerline-s24-55d9857fcc6f"
DISPLAY_NAME = "CD210 Centerline s24"
DESCRIPTOR_FILE = f"{CANDIDATE_REF}.bundle.json"
RECEIPT_FILE = f"{CANDIDATE_REF}.device-report.json"
_ACTIVE = "ModelManager_ActiveBundle"


def _root():
    from iqpilot.system.hardware.hw import Paths
    return Path(Paths.model_root())


def validated_bundle():
    """Rehash on explicit selection; never load model code while browsing."""
    root = _root()
    try:
        bundle = json.loads((root / DESCRIPTOR_FILE).read_text())
        report = json.loads((root / RECEIPT_FILE).read_text())
        if not (bundle['ref'] == CANDIDATE_REF and bundle['runner'] == 'tinygrad'
                and report['runtime_ready'] is True and report['passed'] is True
                and {m['type'] for m in bundle['models']} == {'vision', 'policy'}):
            raise ValueError('candidate is not runtime verified')
        for model in bundle['models']:
            for kind in ('artifact', 'metadata'):
                record = model[kind]
                name = record['fileName']
                if Path(name).name != name:
                    raise ValueError('invalid artifact path')
                expected = record['downloadUri']['sha256']
                if (report['artifacts'][name] != expected
                        or hashlib.sha256((root / name).read_bytes()).hexdigest() != expected):
                    raise ValueError(f'candidate integrity failed: {name}')
        return bundle
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise RuntimeError(f'{DISPLAY_NAME} is unavailable: {error}') from error


def select_candidate(params, *, offroad):
    if not offroad:
        raise RuntimeError('Change models while offroad')
    bundle = validated_bundle()
    # The existing picker owns model changes. No new toggle or Param is needed.
    params.remove('ModelManager_DownloadIndex')
    params.remove('ModelManager_PendingIndex')
    params.remove('ModelRunnerTypeCache')
    params.put(_ACTIVE, bundle)


def is_selected(params):
    bundle = params.get(_ACTIVE)
    return isinstance(bundle, dict) and bundle.get('ref') == CANDIDATE_REF
