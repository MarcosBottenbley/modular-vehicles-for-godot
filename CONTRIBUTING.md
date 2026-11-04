# Contributing

Thanks for helping with Modular Vehicles for Godot. This is version 0 of the contribution
guide; it will grow as the conversion from the GDScript addon progresses.

## Where the work is tracked

Every piece of work is a card on the
[Trello board](https://trello.com/b/MIH0pFKn/modular-vehicles-for-godot). Pick a card from
"Ready", move it to "In Progress" when you push a branch, to "In Review" when you open the PR
(put the PR link on the card), and the owner moves it to "Done" on merge.

## Branches

- One branch per Trello card. Name it after the kind of change and the card's slug:
  `feature/<slug>`, `ci/<slug>` or `docs/<slug>`.
- Always create it from freshly fetched `origin/main`:

  ```
  git fetch origin
  git switch -c feature/<slug> origin/main
  ```

- Never push to `main`. Never force-push a branch that is under review without saying so in
  the PR.

## Pull requests

- One PR per branch, opened against `main` (`gh pr create --base main` works well).
- Keep a PR reviewable in one sitting: roughly 200 to 800 lines of real change. Scene files
  and generated data do not count, but split the work across cards if it grows past that.
- Fill in every section of the PR template:
  - **Summary**: what changed and why.
  - **Trello card**: link to the card.
  - **How verified**: builds, tests (doctest, GUT), demo scenes, platforms you checked.
  - **Docs touched**: files under `docs/`, `doc_classes/` or `CLAUDE.md`, or "none".
  - **Migration impact**: what changes for users of the old GDScript addon, or "none".
  - **Checklist**: branched from fresh `main`, CI green, nothing committed under
    `addons/modular_vehicles/bin/`, `bin/` or `src/gen/`.
- The Build workflow must be green on all matrix jobs (Windows, Linux and macOS, each for
  `editor`, `template_debug` and `template_release`) before review.

## Review and merge

- The owner reviews and merges every PR. Do not merge your own PR.
- Merges are squash merges, so the PR title and description become the commit on `main`.
  Keep them accurate.
- Address review comments with new commits rather than rewriting history, so the reviewer
  can see what changed.

## Definition of done

A card is done when:

1. its PR is merged into `main`,
2. the Trello card is in "Done", and
3. once a `CHANGELOG.md` exists, it has an entry for the change.

## Building and testing

- Build instructions for all three platforms: [docs/building.md](docs/building.md).
- Test instructions (doctest unit tests and GUT scene tests): [docs/testing.md](docs/testing.md).
- Code conventions and repo rules for humans and AI agents: [CLAUDE.md](CLAUDE.md).

## Commit messages

Write a short imperative subject line and, when useful, a body explaining why. Commits
produced with an AI agent carry the attribution trailer that agent is configured with.
