"""Reserve an easy-to-find clean map directory without overwriting earlier runs."""
from pathlib import Path


def create_output_directory(source, output=None):
    source = Path(source).expanduser().resolve()
    explicit = output is not None
    base = Path(output).expanduser().resolve() if explicit else source.with_name(source.name + '_clean')
    if base == source or base in source.parents:
        raise ValueError('Clean output must not replace the input map or its parents')
    candidate = base
    number = 1
    while True:
        try:
            candidate.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            if explicit:
                raise FileExistsError(f'Output already exists; choose a new --output: {base}')
            number += 1
            candidate = base.with_name(f'{base.name}_{number}')
        else:
            (candidate / 'INCOMPLETE').write_text('Semantic map cleaning is still in progress.\n')
            return candidate
