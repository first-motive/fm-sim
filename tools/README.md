# Tools

## Fetch SimReady Assets

`fetch_simready.py` installs named packages of the Hugging Face dataset
[`nvidia/simready-assets`](https://huggingface.co/datasets/nvidia/simready-assets)
into a directory the sim agent's `--assets-dir` mounts into Isaac Sim. It uses the
standard library only.

```bash
uv run --no-project tools/fetch_simready.py --assets-dir /data/fm-assets \
    food_beverage_can_e01 food_canned_beans_a02 food_box_gelatin_a01
```

Those three are what the `tabletop_simready` scenario names.

```
pinned commit -> listing -> staging directory -> hashes and licence checked -> read-only rename
```

- `--revision` takes only a full commit SHA, and the default is the one the scenario
  was measured against.
- Package names and listed paths are checked as hostile, so nothing lands outside the
  assets directory.
- Downloads use https only, to the dataset's host and its content stores, redirects
  included.
- Each file must match the size and hash the host lists at that commit. A failed
  package leaves nothing installed.
- A second run verifies what is installed instead of downloading, and refuses a copy
  that differs from the commit.
- One directory holds one commit. To change commit, use a new directory.

To test it:

```bash
uv run --no-project --with pytest pytest tools
```

## Attribution

The packages are NVIDIA SimReady assets, licensed under
[CC-BY-4.0](https://creativecommons.org/licenses/by/4.0/). This repository names them
and does not redistribute them. The tool installs them unchanged, writes `NOTICE.txt`
with the source, commit and licence into the assets directory, and leaves each
package's own licence text in its `.metadata/LICENSE.txt`.
