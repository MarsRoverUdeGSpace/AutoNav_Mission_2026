# Withdrawn v1.0.0 promotion and 2027 development handoff

Status: recorded 2026-10-04. This is a history/transition note, not a new release or evidence that a fresh Ubuntu 24.04 host was tested.

## What happened

On 2026-06-02, `develop` advanced from `559e36b4d36b60c4d22d5d8dbdbef404545b8007` ("Tune SLAM mapping precision baseline") to `080f3fb11adf0ca2d915aec46368283e01c3d6b9` ("release: promote stable gnss waypoint navigation"). That single promotion commit changed the ROS launch/configuration and added other packages and assets; the change was much broader than simply naming a release. It was merged into `main` at `14ce64fcaf63f886124ff58cadef653c05111721`, whose first parent was `03585d2cd96d4023459bf31b4f431256d22cc54a`. An annotated `v1.0.0` Git tag pointed to the merge. GitHub's Releases API showed **no published Release**; the tag and branch commits were the release-related objects.

Ro chose to withdraw the promotion rather than continue from it: on 2026-10-04, remote and local `develop` were moved back to `559e36b`; remote and local `main` were restored to their pre-merge tip `03585d2`; the `v1.0.0` tag was deleted locally and remotely. Both branch changes and the remote tag deletion were sent in one guarded atomic push, then checked with remote refs and GitHub's branch/tag endpoints. The rest of the remote branches were not rewritten. This was a history rewrite of shared branch tips, **not** a `git revert` commit and not proof of the technical cause of the ROS problems. Collaborators with older clones must resynchronize before pushing to either branch.

Local recovery references, deliberately **not published as branches or tags**, preserve the withdrawn objects in Ro's original clone:

- `refs/backup/autonav-pre-withdraw-develop` -> `080f3fb`
- `refs/backup/autonav-pre-withdraw-main` -> `14ce64f`
- `refs/backup/autonav-pre-withdraw-v1.0.0` -> the old annotated tag object `97ffb81`

Those refs are only in that clone; do not mistake them for remote backups. No `v1.0.0` tag or GitHub Release should be presented as current. The resulting 2026 `develop` history is the base for the container documentation and the 2027 handoff. The 2026 `main` remains its older, pre-release line; it is not the Docker/Jazzy integration branch.

## Known-good scope and next verification

The working Docker prototype uses a Fedora 44 graphical host with an Ubuntu 24.04 / ROS 2 Jazzy container. The host checkout is bind-mounted into the container. The exact first-run, reboot and ROS launch commands, including the Fedora SELinux override, are in [docker-setup.md](docker-setup.md). Ro previously reported that the image built, the two ROS packages built incrementally, Gazebo/RViz opened and a navigation goal succeeded after retries. This is a report of the Fedora run, not validation of a new clone on native Ubuntu 24.04; an RViz GLSL shader-link warning remains worth checking visually. The container's underlying `ros2 launch maya_bringup maya.launch.xml` is invoked after sourcing `/opt/ros/jazzy/setup.bash` **and** the workspace `install/setup.bash` in the same shell. On reboot, re-create the ephemeral X11 cookie with `tools/prepare_gui_env.sh` as the desktop user, start/recreate `jazzy`, and only then use `exec` to launch ROS. A directory accidentally created at the missing Xauthority bind source must be removed safely before rerunning preflight; see the troubleshooting section in the setup guide.

A teammate still needs to validate a clean Ubuntu 24.04 desktop clone: LFS meshes fetched, host preflight passes, image builds, `colcon` reports two packages, Gazebo and RViz display Maya, and `/scan` produces a message. Do not claim the new `Autonomous_Navigation_2027` clone has been exercised merely because it shares Git history or a Dockerfile.

## Transfer to Autonomous_Navigation_2027

Use the **whole 2026 `develop` history**, not a single copied snapshot, as the starting `dev` branch of `git@github.com:MarsRoverUdeGSpace/Autonomous_Navigation_2027.git`. This keeps the commit ancestry and documents why development resumed at `559e36b` before the Docker integration. The destination already has a separate `maya_arm_simulation` branch: leave it and the destination default branch untouched. Do not recreate the withdrawn release tag or push every 2026 branch. Verify that `dev` points to the exact committed handoff revision and that the old destination branches have not moved.

The initial transfer intentionally retains 2026 package names and hard-coded `/workspace/AutoNav_Mission_2026` paths in the Compose/Docker setup. Until these are migrated and tested in the 2027 project, clone the destination `dev` branch into a local directory named `AutoNav_Mission_2026` (or keep an existing checkout with that directory name):

```bash
git clone --branch dev git@github.com:MarsRoverUdeGSpace/Autonomous_Navigation_2027.git AutoNav_Mission_2026
cd AutoNav_Mission_2026
git lfs pull
```

Renaming the host checkout without changing the Compose bind target, working directory, Dockerfile and shell commands is **not** a tested configuration. Treat 2027-specific renaming as a follow-up change, not part of the history-preserving transfer.
