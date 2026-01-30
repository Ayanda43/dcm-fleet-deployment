# DCM Fleet Deployment

This repository contains an automated deployment script for DCM (Distributed Control Manager) fleet systems.

## Overview

The primary deployment script is `deploy-dcm-fleet.sh`. It performs a full system setup including:

- Optional RevPi kernel setup (requires reboot)
- OS package prerequisites and GitHub CLI installation
- ROS2 (Kilted) installation and configuration
- Cloning the DCM control repository (configurable)
- Node.js 20.x installation and application build
- Zenoh router and services setup (via included script in the cloned repo)
- Kiosk display setup (LightDM + Chromium + Openbox)
- Automatic continuation after reboot and state persistence

## New: Selectable DCM repository

You can now choose which DCM repository to clone at deployment time.

Options:

- Default: `tsam` (clones `Ayanda43/tsam-dcm`) — same behaviour as before when no option is provided.
- `corolla` — clones the Corolla repository (the script supports the Git URL `https://github.com/Ayanda43/corrolla-dcm.git`).
- `--repo-url` — provide a custom repository URL or a GitHub `owner/repo` short name. This takes precedence over `--dcm`.

Examples:

- Default (tsam):
  sudo ./deploy-dcm-fleet.sh

- Explicitly choose corolla:
  sudo ./deploy-dcm-fleet.sh --dcm=corolla

- Use a custom repo URL (any git URL or GitHub owner/repo string):
  sudo ./deploy-dcm-fleet.sh --repo-url=https://github.com/Ayanda43/corrolla-dcm.git
  sudo ./deploy-dcm-fleet.sh --repo-url=Ayanda43/tsam-dcm

Notes on cloning behavior:

- If `--repo-url` begins with `http://` or `https://` the script will use `git clone <url> dcm-control`.
- If a short form like `owner/repo` is provided, the script will attempt to use `gh repo clone owner/repo dcm-control` (requires `gh` CLI installed and authenticated for private repos).
- If no option is provided the script defaults to `Ayanda43/tsam-dcm`.
- The chosen repository is cloned into `/opt/commander/fleet/dcm-control` (same directory regardless of choice). If you prefer separate directories per repo (for example, `dcm-tsam` and `dcm-corolla`) please open an issue or request that change.

## Usage

Basic interactive run:

  sudo ./deploy-dcm-fleet.sh

Automated (non-interactive):

  sudo ./deploy-dcm-fleet.sh --auto

Choose DCM repo (examples repeated here):

  sudo ./deploy-dcm-fleet.sh --dcm=corolla
  sudo ./deploy-dcm-fleet.sh --repo-url=Ayanda43/tsam-dcm

Continue after a scheduled reboot (internal use):

  sudo ./deploy-dcm-fleet.sh --continue

## Requirements

- Debian/Ubuntu-based system (script uses apt and snaps)
- `sudo` or root privileges to run the script
- Internet connectivity
- If cloning private repositories with the `gh` CLI, authenticate `gh` beforehand (or ensure SSH/git credentials available for `git clone`).

## Files of interest

- `deploy-dcm-fleet.sh` - main deployment orchestrator (supports `--dcm` and `--repo-url`)
- `scripts/dcm_zenoh_setup.sh` - (expected inside the cloned DCM repo) sets up Zenoh router and related services

## Troubleshooting

- Check deployment logs: `/var/log/fleet-deployment.log`
- Check saved state to re-run from the correct phase: `/var/lib/commander/fleet/deployment_state`
- If the repo fails to clone, verify `gh` CLI is installed and authenticated or that the provided git URL is accessible.

## Contributing

Please open issues or pull requests against this repository. If you'd like the script to clone into distinct directories per repo (instead of overwriting `dcm-control`), mention that in the issue and provide the desired naming scheme.

---

This README was updated to document the new `--dcm` and `--repo-url` options for selecting the DCM repository during deployment.
