# AGENTS.md

<!-- fm-render:begin agents-invariants sha256:ae8e770a702d661078394c75dae8593ec122d51514bc90016ef0f3b8fd8194b1 — rendered by the First Motive render plane — edit the upstream source, not this file -->
## First Motive Invariants

These five hold in every First Motive repo. Break one and the review, the CI
check, or the next machine catches it — usually all three.

- **Names.** Repos, packages, and hosts are `fm-<kebab>`; Python modules are
  `fm_<snake>`. A name that does not carry the `fm` prefix is unreachable to the
  `fm` CLI and to the tooling that discovers repos by prefix.
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
  push that takes it is reported on the branch by a tripwire workflow.
- **Python through uv.** `uv run`, `uv add`, `uv sync` — never bare `python`,
  `pip`, `poetry`, or `virtualenv`. A bare invocation resolves against whatever
  interpreter the shell happens to have, which is why "works on my machine"
  reports are almost always this.
<!-- fm-render:end agents-invariants -->
