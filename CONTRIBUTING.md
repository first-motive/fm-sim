# Contributing

Thanks for contributing. This repo uses an owner-free-on-main model: the owner
pushes to `main` directly, everyone else works on a branch and opens a pull
request for the owner to merge. The owner is set in
[`.github/CODEOWNERS`](.github/CODEOWNERS).

## Workflow

```text
owner:   push main
others:  branch -> PR -> owner merges
```

The merge-to-main rules apply to the owner only. If you are not the owner, you
branch and open a PR — you do not merge.

## Branch Naming

Name branches `prefix/short-phrase`, where the prefix matches the commit prefix
list below and the phrase is a kebab-case summary.

```text
feat/license-gate
fix/empty-manifest-crash
docs/contributing-guide
```

- Lowercase, hyphen-separated.
- No `:` or spaces (invalid in git refs).
- Short — the branch name is a label, not a description.

## Commit Format

Commits are subject-line-only: `prefix: phrase`. Use a lowercase imperative
phrase, no trailing period, no body. Add a `Co-Authored-By` trailer only when a
commit genuinely had more than one author.

| Prefix     | Use for                                              | Example                          |
| ---------- | --------------------------------------------------- | -------------------------------- |
| `init`     | First commit of a repo (bootstrap only, never after) | `init: scaffold project`         |
| `feat`     | New behavior or content                             | `feat: add license gate`         |
| `fix`      | Bug fix or content correction                       | `fix: handle empty manifest`     |
| `docs`     | Documentation only                                  | `docs: document github pipeline` |
| `refactor` | Behavior-preserving restructure                     | `refactor: extract normalizer`   |
| `chore`    | Tooling, deps, housekeeping                         | `chore: bump lockfile`           |

Pick the narrowest prefix that fits. If a change spans two, split the commit.

Avoid capitalized or past-tense subjects (`feat: Added gate.`) and vague
non-standard prefixes (`update: stuff`).

## Pull Requests

- One logical change per PR. Split unrelated work.
- Fill the PR template: **what** changed, **why**, and how you **tested** it.
- Keep the branch current with `main` before requesting a merge.

## Repo Conventions

### Naming

- **Repo** — kebab-case (`fm-ros2`), matching the GitHub repo and the
  `first-motive` org.
- **Packages** — snake_case (`fm_bringup`, `fm_data`). ROS2 requires snake_case
  package names; the inner Python module, the `resource/` marker, and the
  `package.xml` `<name>` all match it exactly.

## Tests

Run the test suite before opening a PR. This is a ROS2 (Humble) workspace built
with colcon:

```bash
colcon build --symlink-install
colcon test --packages-select $(colcon list --names-only | grep '^fm_')
colcon test-result --verbose
```
