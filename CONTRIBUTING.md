# Contributing

Thanks for contributing.

## Workflow

<!-- fm-render:begin contributing-workflow sha256:4a3ac9a4c1cbb5995c5a0e4d5b6beb580f50090c14bc5170cd21c84ee4635dd0 — rendered by the First Motive render plane — edit the upstream source, not this file -->
Work reaches `main` by merging a pull request with green checks — never by
pushing to it. This holds for everyone, the owner included. The rendered
`.fm/hooks/pre-push` refuses a direct push, a tripwire workflow files an issue
against one that arrives from an unguarded clone, and ADR 0001 in
`first-motive/.github` records the decision.

```text
everyone:  branch -> PR -> green checks -> merge
```

`FM_ALLOW_MAIN_PUSH=1` is the loud escape for an emergency; a push that takes
it is still reported by the tripwire. The repo owner is set in
[`.github/CODEOWNERS`](.github/CODEOWNERS).
<!-- fm-render:end contributing-workflow -->

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

Run the test suite before opening a PR. This is a ROS2 workspace built
with colcon:

```bash
colcon build --symlink-install
colcon test --packages-select $(colcon list --names-only | grep '^fm_')
colcon test-result --verbose
```
