# AGENTS.md

<!-- fm-render:begin agents-invariants sha256:311959fea747960e045bbf2fcf4e3314930fe25555e9df4931c97b4c0f7017f0 — rendered by the First Motive render plane — edit the upstream source, not this file -->
## First Motive Invariants

These five guide First Motive repos. Existing name exceptions are listed below.
The review, CI check, or next machine usually catches other violations.

- **Names.** New First Motive repos and packages use `fm-<kebab>`; Python
  modules use `fm_<snake>`. The existing organization repos `.github`,
  `.github-private`, `anvil-loader`, `anvil-embodied-ai`, and `tactile-gate` are
  named exceptions. Hosts normally use `fm-<kebab>`; Rune is the existing
  `adiis-mac-mini` exception. Check the `fm` registry before assuming a repo
  or host is discoverable from its name.
- **Config, never source.** Anything that differs per host — hostname, role,
  workspace path, transport, device IDs — is read from `machine.json`, never
  typed into a script, unit file, or launch file. A hardcoded host value works on
  exactly one machine and silently breaks the rest of the fleet.
- **Commits.** Subject line only: `prefix: phrase`, lowercase imperative, no
  body, no trailers. Prefixes: `init`, `feat`, `fix`, `docs`, `refactor`,
  `chore`. A commit body is dropped by the repo's hook, so anything explained
  there is lost.
- **Main through a pull request.** Work reaches the default branch by merging a
  PR with green checks, never by pushing to it. The rendered `.fm/hooks/pre-push`
  refuses a direct push; `FM_ALLOW_MAIN_PUSH=1` is the one-command escape, and a
  push that takes it is reported on the branch by a tripwire workflow. An agent
  ships its own work end to end: push the branch, open the PR, watch the
  checks, merge. `gh pr merge --admin` is allowed when a required review is the
  only blocker — never on a red or pending check, never with a force-push.
- **Python through uv.** `uv run`, `uv add`, `uv sync` — never bare `python`,
  `pip`, `poetry`, or `virtualenv`. A bare invocation resolves against whatever
  interpreter the shell happens to have, which is why "works on my machine"
  reports are almost always this.
<!-- fm-render:end agents-invariants -->
